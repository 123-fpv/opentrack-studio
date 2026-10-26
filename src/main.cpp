#include "window.h"
#include <QApplication>
#include <QFile>
#include <QIcon>
#include <QLibrary>
#include <QListWidget>
#include <QPalette>
#include <QScreen>
#include <QStyleFactory>
#include <QTimer>
#include <algorithm>

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
    app.setApplicationVersion("0.1.3");
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
    const QStringList args = app.arguments();
    ot::StudioWindow window;
    if (args.size() >= 3 && (args[1] == "--render-demo" || args[1] == "--smoke-test")) {
        // Automated screenshots must not take keyboard focus from another app.
        window.setAttribute(Qt::WA_ShowWithoutActivating);
        for (auto *screen : app.screens())
            if (screen != app.primaryScreen()) {
                const QRect available = screen->availableGeometry();
                window.resize(std::min(window.width(), available.width() - 40),
                              std::min(window.height(), available.height() - 40));
                window.move(available.topLeft() + QPoint(20, 20));
                break;
            }
    }
    window.show();
    applyWindowCaption(window);
    int result = 0;
    if (args.size() >= 3 && args[1] == "--smoke-test") {
        QTimer::singleShot(200, &window, [&] {
            result = window.runSmokeTest(args[2]) ? 0 : 1;
            app.exit(result);
        });
    } else if (args.size() >= 3 && args[1] == "--render-demo") {
        if (args.size() >= 4 && !window.openPath(args[3]))
            return 1;
        if (auto *order = window.findChild<QListWidget *>("orderList"))
            order->setCurrentRow(0);
        QTimer::singleShot(800, &window, [&] {
            result = window.screen()->grabWindow(window.winId()).save(args[2]) ? 0 : 1;
            app.exit(result);
        });
    } else if (args.size() >= 2) {
        window.openPath(args[1]);
    }
    return app.exec();
}
