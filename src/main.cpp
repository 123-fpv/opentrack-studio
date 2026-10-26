#include "window.h"
#include <QApplication>
#include <QFile>
#include <QIcon>
#include <QLibrary>
#include <QPalette>
#include <QScreen>
#include <QStyleFactory>
#include <QTimer>

namespace {
void applyWindowCaption(QWidget &window) {
#ifdef Q_OS_WIN
    if (QGuiApplication::platformName() != "windows")
        return;
    QLibrary dwm("dwmapi");
    using SetAttribute = long(__stdcall *)(void *, unsigned long, const void *, unsigned long);
    auto setAttribute = reinterpret_cast<SetAttribute>(dwm.resolve("DwmSetWindowAttribute"));
    if (!setAttribute)
        return;
    auto *handle = reinterpret_cast<void *>(window.winId());
    const int dark = 1;
    const quint32 caption = 0x00111111, text = 0x00fafafa;
    setAttribute(handle, 20, &dark, sizeof(dark));
    setAttribute(handle, 35, &caption, sizeof(caption));
    setAttribute(handle, 36, &text, sizeof(text));
#endif
}
} // namespace

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    app.setApplicationName("OpenTrack Studio");
    app.setApplicationVersion("0.1.1");
    app.setOrganizationName("OpenTrackStudio");
    app.setStyle(QStyleFactory::create("Fusion"));
    app.setFont(QFont("Microsoft YaHei UI", 9));
    app.setWindowIcon(QIcon(":/icons/app.png"));
    QPalette palette = app.palette();
    palette.setColor(QPalette::Highlight, QColor("#222222"));
    palette.setColor(QPalette::HighlightedText, Qt::white);
    palette.setColor(QPalette::Link, QColor("#333333"));
    palette.setColor(QPalette::LinkVisited, QColor("#666666"));
    app.setPalette(palette);
    QFile theme(":/theme/studio.qss");
    if (theme.open(QIODevice::ReadOnly))
        app.setStyleSheet(QString::fromUtf8(theme.readAll()));
    ot::StudioWindow window;
    window.show();
    applyWindowCaption(window);
    const QStringList args = app.arguments();
    int result = 0;
    if (args.size() >= 3 && args[1] == "--smoke-test") {
        QTimer::singleShot(200, &window, [&] {
            result = window.runSmokeTest(args[2]) ? 0 : 1;
            app.exit(result);
        });
    } else if (args.size() >= 3 && args[1] == "--render-demo") {
        QTimer::singleShot(800, &window, [&] {
            result = window.screen()->grabWindow(window.winId()).save(args[2]) ? 0 : 1;
            app.exit(result);
        });
    } else if (args.size() >= 2) {
        window.openPath(args[1]);
    }
    return app.exec();
}
