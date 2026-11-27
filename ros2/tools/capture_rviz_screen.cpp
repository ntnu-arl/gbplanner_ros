// Capture the existing X11 virtual desktop containing the native RViz window.
// Build with Qt5Widgets pkg-config flags; no Python Qt package is required.
#include <cstdio>
#include <QApplication>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPixmap>
#include <QScreen>
#include <QTextStream>

int main(int argc, char* argv[]) {
  if (argc != 2) {
    QTextStream(stderr) << "Usage: capture_rviz_screen OUTPUT.png\n";
    return 2;
  }
  if (qEnvironmentVariableIsEmpty("DISPLAY")) {
    QTextStream(stderr) << "An existing X11 DISPLAY is required\n";
    return 2;
  }
  const QString output = QString::fromLocal8Bit(argv[1]);
  QApplication application(argc, argv);
  QScreen* screen = QGuiApplication::primaryScreen();
  if (!screen) {
    QTextStream(stderr) << "No Qt screen is available\n";
    return 1;
  }
  // Window ID 0 is the complete screen/root window, not a recreated figure.
  const QPixmap image = screen->grabWindow(0);
  if (image.isNull()) {
    QTextStream(stderr) << "Qt could not capture the existing desktop\n";
    return 1;
  }
  QFile file(output);
  if (!file.open(QIODevice::WriteOnly | QIODevice::NewOnly)) {
    QTextStream(stderr) << "Cannot create new screenshot: " << file.errorString() << '\n';
    return 1;
  }
  if (!image.save(&file, "PNG")) {
    QTextStream(stderr) << "Could not encode captured desktop as PNG\n";
    return 1;
  }
  file.close();
  const QJsonObject record{{"output", output}, {"width", image.width()},
                           {"height", image.height()}, {"screen", screen->name()},
                           {"device_pixel_ratio", image.devicePixelRatio()}};
  QTextStream(stdout) << QJsonDocument(record).toJson(QJsonDocument::Compact) << '\n';
  return 0;
}
