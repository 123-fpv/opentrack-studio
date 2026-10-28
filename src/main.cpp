#include "language.h"
#include "window.h"
#include <QApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
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
    app.setApplicationVersion("0.1.7");
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
    QStringList args = app.arguments();
    // Automated runs use their own INI settings, never the user's saved locale.
    for (const auto &mode :
         {QString("--smoke-test"), QString("--language-smoke-test"), QString("--render-demo")}) {
        const int at = args.indexOf(mode);
        if (at < 0 || at + 1 >= args.size())
            continue;
        const QString output = mode == "--render-demo" ? QFileInfo(args[at + 1]).absolutePath()
                                                       : QFileInfo(args[at + 1]).absoluteFilePath();
        QDir().mkpath(output + "/settings");
        QSettings::setDefaultFormat(QSettings::IniFormat);
        QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, output + "/settings");
        break;
    }
    ot::i18n::initialize();
    for (int i = 1; i < args.size();) {
        if (args[i] == "--lang" && i + 1 < args.size()) {
            ot::i18n::setLanguage(args[i + 1]);
            args.removeAt(i);
            args.removeAt(i);
        } else if (args[i].startsWith("--lang=")) {
            ot::i18n::setLanguage(args[i].mid(7));
            args.removeAt(i);
        } else
            ++i;
    }
    ot::StudioWindow window;
    if (args.size() >= 3 && (args[1] == "--render-demo" || args[1] == "--smoke-test" ||
                             args[1] == "--language-smoke-test")) {
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
    if (args.size() >= 3 && args[1] == "--language-smoke-test") {
        QTimer::singleShot(200, &window, [&] {
            result = window.runLanguageSmokeTest(args[2]) ? 0 : 1;
            app.exit(result);
        });
    } else if (args.size() >= 3 && args[1] == "--smoke-test") {
        QTimer::singleShot(200, &window, [&] {
            result = window.runSmokeTest(args[2]) ? 0 : 1;
            app.exit(result);
        });
    } else if (args.size() >= 3 && args[1] == "--render-demo") {
        if (args.size() >= 4 && !window.openPath(args[3]))
            return 1;
        if (auto *order = window.findChild<QListWidget *>("routeOrderList")) {
            const int requested = args.size() >= 5 ? args[4].toInt() - 1 : 0;
            order->setCurrentRow(requested >= 0 && requested < order->count() ? requested : 0);
        }
        QTimer::singleShot(800, &window, [&] {
            result = window.screen()->grabWindow(window.winId()).save(args[2]) ? 0 : 1;
            app.exit(result);
        });
    } else if (args.size() >= 2) {
        window.openPath(args[1]);
    }
    return app.exec();
}
