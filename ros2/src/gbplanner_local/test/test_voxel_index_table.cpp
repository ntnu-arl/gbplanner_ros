#include "gbplanner_local/voxel_index_table.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {
using Table = gbplanner2::VoxelIndexTable<std::uint64_t>;
using Key = Table::Key;

std::vector<Key> bucketKeys(std::size_t bucket, std::size_t mask, std::size_t count) {
  std::vector<Key> keys;
  for (int x = 0; x < 10000 && keys.size() < count; ++x) {
    Key key{x, -7, 11};
    if ((gbplanner2::VoxelIndexSet::hash(key) & mask) == bucket) keys.push_back(key);
  }
  return keys;
}

void expectRecords(const Table& table, const std::map<Key, std::uint64_t>& reference,
                   const std::vector<Key>& keys) {
  EXPECT_EQ(table.size(), reference.size());
  for (const auto& key : keys) {
    const auto found = reference.find(key);
    const auto* value = table.find(key);
    if (found == reference.end()) {
      EXPECT_EQ(value, nullptr);
    } else {
      ASSERT_NE(value, nullptr);
      EXPECT_EQ(*value, found->second);
    }
  }
}

// SparseVoxelMap stores this kind of private nested value. Instantiating the
// table before Owner's definition closes must not inspect its default ctor.
class Owner {
  struct Cell { int status{0}; double distance{0.}, weight{0.}; };
  gbplanner2::VoxelIndexTable<Cell> cells_;
 public:
  void set(const Key& key) {
    auto& cell = cells_[key];
    EXPECT_EQ(cell.status, 0);
    EXPECT_DOUBLE_EQ(cell.distance, 0.);
    EXPECT_DOUBLE_EQ(cell.weight, 0.);
    cell = Cell{1, 2., 3.};
  }
  double distance(const Key& key) const { return cells_.find(key)->distance; }
};

bool reject_default = false;
struct ThrowingDefault {
  std::uint64_t value{0};
  ThrowingDefault() {
    if (reject_default) throw std::runtime_error("default construction rejected");
  }
};
static_assert(std::is_trivially_copyable<ThrowingDefault>::value, "fixture remains trivially copyable");
static_assert(noexcept(std::declval<Table&>().find(std::declval<const Key&>())), "mutable find is noexcept");
static_assert(noexcept(std::declval<const Table&>().find(std::declval<const Key&>())), "const find is noexcept");
static_assert(noexcept(std::declval<Table&>().erase(std::declval<const Key&>())), "erase is noexcept");
}  // namespace

TEST(VoxelIndexTable, EmptyDefaultValuesAndConstQueries) {
  Table table;
  const Key key{1, -2, 3}, missing{1, -2, 4};
  EXPECT_EQ(table.size(), 0u);
  EXPECT_EQ(table.find(key), nullptr);
  EXPECT_FALSE(table.erase(key));
  table.reserve(0);
  table.reserve(6);
  EXPECT_EQ(table[key], 0u);
  table[key] = 17;
  const auto* address = table.find(key);
  const Table& view = table;
  for (int query = 0; query < 32; ++query) {
    EXPECT_EQ(view.find(key), address);
    EXPECT_EQ(view.find(missing), nullptr);
  }
  EXPECT_EQ(table.size(), 1u);
  EXPECT_EQ(*address, 17u);
  EXPECT_EQ(&table[key], address);
  Owner owner;
  owner.set(key);
  EXPECT_DOUBLE_EQ(owner.distance(key), 2.);
}

TEST(VoxelIndexTable, WrappingCollisionClusterRepairsMovedEntry) {
  const auto keys = bucketKeys(7, 7, 6);
  ASSERT_EQ(keys.size(), 6u);
  for (const std::size_t victim : {0u, 2u, 5u}) {
    Table table;
    table.reserve(6);  // Eight hash slots; the cluster crosses slot 7 -> 0.
    std::map<Key, std::uint64_t> expected;
    for (std::size_t i = 0; i < keys.size(); ++i) table[keys[i]] = expected[keys[i]] = 100 + i;
    const auto* removed_address = table.find(keys[victim]);
    ASSERT_TRUE(table.erase(keys[victim]));
    expected.erase(keys[victim]);
    if (victim != keys.size() - 1) {
      EXPECT_EQ(table.find(keys.back()), removed_address);
    }
    EXPECT_FALSE(table.erase(keys[victim]));
    expectRecords(table, expected, keys);
    for (std::size_t round = 0; round < 48; ++round) {
      const auto& key = keys[round % keys.size()];
      EXPECT_EQ(table.erase(key), expected.erase(key) != 0);
      table[key] = expected[key] = 200 + round;
      expectRecords(table, expected, keys);
    }
    for (const auto& key : keys) EXPECT_EQ(table.erase(key), expected.erase(key) != 0);
    EXPECT_EQ(table.size(), 0u);
    table[keys.front()] = 9;
    EXPECT_EQ(*table.find(keys.front()), 9u);
  }
}

TEST(VoxelIndexTable, BackwardShiftSkipsEntriesAlreadyAtHome) {
  const auto wrapping = bucketKeys(7, 7, 2);
  const auto at_zero = bucketKeys(0, 7, 1);
  const auto at_one = bucketKeys(1, 7, 1);
  ASSERT_EQ(wrapping.size(), 2u);
  ASSERT_EQ(at_zero.size(), 1u);
  ASSERT_EQ(at_one.size(), 1u);
  const std::vector<Key> keys{wrapping[0], at_zero[0], wrapping[1], at_one[0]};
  Table table;
  table.reserve(6);
  std::map<Key, std::uint64_t> expected;
  for (std::size_t i = 0; i < keys.size(); ++i) table[keys[i]] = expected[keys[i]] = i + 1;
  // Deleting slot 7 must skip the home-at-zero entry, then shift the displaced
  // home-at-seven entry from slot 1 and repair the entry behind it as well.
  for (const auto& key : keys) {
    EXPECT_TRUE(table.erase(key));
    expected.erase(key);
    expectRecords(table, expected, keys);
  }
}

TEST(VoxelIndexTable, FullIntegerKeysRemainDistinctWhenTheirHashesAreIdentical) {
  constexpr int low = std::numeric_limits<int>::min();
  constexpr int high = std::numeric_limits<int>::max();
  constexpr int period = 1 << 21;
  const std::vector<Key> keys{{low, 7, -13}, {low + period, 7, -13},
                              {low, 7 + period, -13}, {low, 7, -13 + period},
                              {high, low, high}};
  for (std::size_t i = 1; i < 4; ++i)
    ASSERT_EQ(gbplanner2::VoxelIndexSet::hash(keys[0]), gbplanner2::VoxelIndexSet::hash(keys[i]));
  Table table;
  std::map<Key, std::uint64_t> expected;
  for (std::size_t i = 0; i < keys.size(); ++i) table[keys[i]] = expected[keys[i]] = i + 10;
  expectRecords(table, expected, keys);
  EXPECT_TRUE(table.erase(keys[1]));
  expected.erase(keys[1]);
  expectRecords(table, expected, keys);
}

TEST(VoxelIndexTable, GrowthCopyMoveAndMovedFromReuse) {
  Table original;
  std::vector<Key> keys;
  std::map<Key, std::uint64_t> expected;
  for (int i = 0; i < 96; ++i) {
    keys.push_back({i, -i, i * 7});
    original[keys.back()] = expected[keys.back()] = i + 20;
  }
  original.reserve(500);
  original.reserve(1);  // Smaller requests preserve the populated table.
  expectRecords(original, expected, keys);
  Table copied(original);
  copied[keys.front()] = 999;
  EXPECT_EQ(*original.find(keys.front()), expected[keys.front()]);
  EXPECT_TRUE(copied.erase(keys.back()));
  ASSERT_NE(original.find(keys.back()), nullptr);
  Table assigned;
  assigned[{-9, -8, -7}] = 4;
  assigned = original;
  auto* self = &assigned;
  assigned = *self;
  EXPECT_EQ(assigned.find({-9, -8, -7}), nullptr);
  expectRecords(assigned, expected, keys);
  Table moved(std::move(assigned));
  EXPECT_EQ(assigned.size(), 0u);
  EXPECT_EQ(assigned.find(keys.front()), nullptr);
  assigned[keys.front()] = 8;
  EXPECT_EQ(*assigned.find(keys.front()), 8u);
  expectRecords(moved, expected, keys);
  copied = std::move(moved);
  EXPECT_EQ(moved.size(), 0u);
  moved.reserve(6);
  moved[{4, 5, 6}] = 7;
  EXPECT_EQ(*moved.find({4, 5, 6}), 7u);
  expectRecords(copied, expected, keys);
  copied = Table{};
  EXPECT_EQ(copied.size(), 0u);
  EXPECT_EQ(copied.find(keys.front()), nullptr);

  gbplanner2::VoxelIndexTable<Key> aliases;
  aliases[{1, 0, 0}] = Key{9, 8, 7};
  aliases[{2, 0, 0}] = Key{};
  aliases[{3, 0, 0}] = Key{};
  // The fourth insertion reallocates entries while its key aliases a value.
  aliases[*aliases.find({1, 0, 0})] = Key{4, 5, 6};
  ASSERT_NE(aliases.find({9, 8, 7}), nullptr);
  EXPECT_EQ((*aliases.find({9, 8, 7})), (Key{4, 5, 6}));
}

TEST(VoxelIndexTable, OverflowAndThrowingDefaultPreserveTheExistingTable) {
  Table table;
  const Key key{1, 2, 3};
  table[key] = 19;
  const auto* address = table.find(key);
  EXPECT_THROW(table.reserve(std::numeric_limits<std::size_t>::max()), std::length_error);
  if constexpr (sizeof(std::size_t) > sizeof(std::uint32_t)) {
    EXPECT_THROW(table.reserve(static_cast<std::size_t>(
        std::uint64_t{std::numeric_limits<std::uint32_t>::max()} + 1)), std::length_error);
  }
  EXPECT_EQ(table.size(), 1u);
  EXPECT_EQ(table.find(key), address);
  EXPECT_EQ(*address, 19u);

  gbplanner2::VoxelIndexTable<ThrowingDefault> values;
  for (int i = 0; i < 3; ++i) values[{i, 0, 0}].value = i + 1;
  const auto* previous = values.find({0, 0, 0});
  reject_default = true;
  EXPECT_THROW((values[Key{9, 0, 0}]), std::runtime_error);
  reject_default = false;
  EXPECT_EQ(values.size(), 3u);
  EXPECT_EQ(values.find({0, 0, 0}), previous);
  EXPECT_EQ(previous->value, 1u);
  EXPECT_EQ(values.find({9, 0, 0}), nullptr);
  values[{9, 0, 0}].value = 42;
  EXPECT_EQ(values.find({9, 0, 0})->value, 42u);
}

TEST(VoxelIndexTable, RandomOperationsMatchOrderedMap) {
  std::mt19937 random(0x802);
  std::vector<Key> keys;
  for (int i = 0; i < 192; ++i)
    keys.push_back({i % 16 + (i / 64) * (1 << 21), (i / 16) % 4, -3});
  Table table;
  std::map<Key, std::uint64_t> reference;
  for (int operation = 0; operation < 2048; ++operation) {
    const Key& key = keys[random() % keys.size()];
    switch (random() % 5) {
      case 0:
      case 1: {
        const std::uint64_t value = random();
        table[key] = reference[key] = value;
        break;
      }
      case 2:
        EXPECT_EQ(table.erase(key), reference.erase(key) != 0);
        break;
      case 3:
        expectRecords(table, reference, {key});
        break;
      case 4:
        table.reserve(random() % 256);
        break;
    }
    if (operation % 64 == 0) expectRecords(table, reference, keys);
  }
  expectRecords(table, reference, keys);
}
