#include "gbplanner_local/map_update_sequence.hpp"
#include <gtest/gtest.h>
using gbplanner2::MapUpdateSequence;

TEST(MapUpdateSequence, ArtificialUpdatesNeverCreateAnObservation){
  MapUpdateSequence tracker;
  MapUpdateSequence::Update initial{"mapper-a",1,1,0,0,"augmentation"};
  EXPECT_TRUE(tracker.newer(initial));EXPECT_FALSE(tracker.hasNewObservation(initial));
  tracker.commit(initial);
  MapUpdateSequence::Update scan{"mapper-a",1,2,1,1000000000,"observation"};
  EXPECT_TRUE(tracker.newer(scan));EXPECT_TRUE(tracker.hasNewObservation(scan));tracker.commit(scan);
  auto artificial=scan;artificial.revision=3;artificial.kind="augmentation";
  EXPECT_TRUE(tracker.newer(artificial));EXPECT_FALSE(tracker.hasNewObservation(artificial));
  tracker.commit(artificial);EXPECT_FALSE(tracker.newer(scan));
  // An atomic snapshot may include a genuine scan not separately received.
  auto skipped=artificial;skipped.revision=5;skipped.observation=2;skipped.observation_stamp_ns=2000000000;
  EXPECT_TRUE(tracker.newer(skipped));EXPECT_TRUE(tracker.hasNewObservation(skipped));
}

TEST(MapUpdateSequence, TransactionAndEpochPreventMapRollback){
  MapUpdateSequence tracker;
  MapUpdateSequence::Update scan{"mapper-a",1,4,2,2000000000,"observation"};
  ASSERT_TRUE(tracker.newer(scan));tracker.commit(scan);
  auto malformed=scan;malformed.revision=5;malformed.observation_stamp_ns=3000000000;
  EXPECT_THROW(tracker.newer(malformed),std::invalid_argument);
  auto parsed_later=scan;parsed_later.revision=5;
  EXPECT_TRUE(tracker.newer(parsed_later)); // no commit on failed map parsing
  EXPECT_EQ(tracker.current().revision,4u);
  MapUpdateSequence::Update reset{"mapper-a",2,5,0,0,"reset"};
  ASSERT_TRUE(tracker.newer(reset));EXPECT_FALSE(tracker.hasNewObservation(reset));tracker.commit(reset);
  EXPECT_FALSE(tracker.newer(parsed_later)); // old epoch cannot return later
  auto next=reset;next.revision=6;next.observation=1;next.observation_stamp_ns=1000000000;next.kind="observation";
  EXPECT_TRUE(tracker.newer(next));EXPECT_TRUE(tracker.hasNewObservation(next));
}

TEST(MapUpdateSequence, IndependentSensorsCanIntegrateOlderAcquisitionTimes){
  MapUpdateSequence tracker;
  MapUpdateSequence::Update front{"mapper-a",1,1,1,10000000000,"observation"};
  ASSERT_TRUE(tracker.newer(front));tracker.commit(front);
  // The rear sensor's delayed TF can make its older scan finish integration
  // after the front sensor. Its acquisition time must remain visible as-is.
  MapUpdateSequence::Update rear{"mapper-a",1,2,2,9000000000,"observation"};
  ASSERT_TRUE(tracker.newer(rear));EXPECT_TRUE(tracker.hasNewObservation(rear));
  tracker.commit(rear);
  EXPECT_EQ(tracker.current().observation_stamp_ns,9000000000);

  auto augmented=rear;augmented.revision=3;augmented.kind="augmentation";
  ASSERT_TRUE(tracker.newer(augmented));EXPECT_FALSE(tracker.hasNewObservation(augmented));
  tracker.commit(augmented);
  EXPECT_FALSE(tracker.newer(front));

  auto fabricated_stamp=augmented;fabricated_stamp.revision=4;
  fabricated_stamp.observation_stamp_ns=10000000000;
  EXPECT_THROW(tracker.newer(fabricated_stamp),std::invalid_argument);
  fabricated_stamp.observation_stamp_ns=8000000000;
  EXPECT_THROW(tracker.newer(fabricated_stamp),std::invalid_argument);
}

TEST(MapUpdateSequence, SkippedResetEpochCannotReplaceNewerServiceSnapshot){
  MapUpdateSequence tracker;
  MapUpdateSequence::Update initial{"mapper-a",1,10,4,10000000000,"observation"};
  ASSERT_TRUE(tracker.newer(initial));tracker.commit(initial);
  // The service snapshot arrives before queued topic traffic from an
  // intermediate reset. That intermediate epoch need never have been seen.
  MapUpdateSequence::Update newest{"mapper-a",3,15,0,0,"augmentation"};
  ASSERT_TRUE(tracker.newer(newest));EXPECT_FALSE(tracker.hasNewObservation(newest));
  tracker.commit(newest);
  MapUpdateSequence::Update delayed{"mapper-a",2,13,0,0,"reset"};
  EXPECT_FALSE(tracker.newer(delayed));
  delayed.revision=16;EXPECT_FALSE(tracker.newer(delayed));
  EXPECT_EQ(tracker.current().epoch,3u);

  auto observed=newest;observed.revision=16;observed.observation=1;
  observed.observation_stamp_ns=11000000000;observed.kind="observation";
  ASSERT_TRUE(tracker.newer(observed));EXPECT_TRUE(tracker.hasNewObservation(observed));
  tracker.commit(observed);
  // Even if reset publication was skipped, epoch-four scan one is a new
  // observation relative to epoch-three scan one.
  auto after_reset=observed;after_reset.epoch=4;after_reset.revision=18;
  ASSERT_TRUE(tracker.newer(after_reset));EXPECT_TRUE(tracker.hasNewObservation(after_reset));
  after_reset.revision=16;EXPECT_FALSE(tracker.newer(after_reset));
}

TEST(MapUpdateSequence, RestartRetiresOnlyThePreviousMapperInstance){
  MapUpdateSequence tracker;
  MapUpdateSequence::Update previous{"mapper-a",3,15,2,2000000000,"observation"};
  ASSERT_TRUE(tracker.newer(previous));tracker.commit(previous);
  MapUpdateSequence::Update restarted{"mapper-b",1,1,0,0,"snapshot"};
  ASSERT_TRUE(tracker.newer(restarted));EXPECT_FALSE(tracker.hasNewObservation(restarted));
  tracker.commit(restarted);
  ++previous.revision;EXPECT_FALSE(tracker.newer(previous));
  auto invalid=restarted;invalid.epoch=0;invalid.revision=2;
  EXPECT_THROW(tracker.newer(invalid),std::invalid_argument);
}
