#include "window.h"
#include <QApplication>
#include <QFile>
#include <QIcon>
#include <QScreen>
#include <QStyleFactory>
#include <QTimer>

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    app.setApplicationName("OpenTrack Studio");
    app.setApplicationVersion("0.1.0");
    app.setOrganizationName("OpenTrackStudio");
    app.setStyle(QStyleFactory::create("Fusion"));
    app.setFont(QFont("Microsoft YaHei UI", 9));
    app.setWindowIcon(QIcon(":/icons/app.png"));
    app.setStyleSheet(R"(
        * {font-family:'Microsoft YaHei UI';} QWidget{color:#30445f;background:#ffffff;} QLabel{background:transparent;}
        QMainWindow,QWidget#center{background:#eef3f9;} QFrame#header{background:#132640;} QFrame#header QLabel{background:transparent;}
        QLabel#logo{background:#2a7bfa;color:white;font-size:21px;font-weight:800;border-radius:11px;qproperty-alignment:AlignCenter;}
        QLabel#brand{color:#f7faff;font-size:20px;font-weight:700;} QLabel#brandSub{color:#9bb2cf;font-size:10px;}
        QLabel#headerTab{color:#b9d6ff;font-size:12px;padding:8px 0;border-bottom:2px solid #5b9fff;} QLabel#localBadge{color:#8bdac3;font-size:11px;padding:8px;}
        QFrame#toolbar{background:#ffffff;border-bottom:1px solid #dce5f0;} QFrame#sidebar,QFrame#rightSidebar{background:#ffffff;} QScrollArea{border:0;}
        QPushButton{background:#ffffff;border:1px solid #dce5f0;border-radius:6px;padding:8px 11px;font-size:11px;min-height:17px;}
        QPushButton:hover{background:#edf5ff;border-color:#a9caf5;} QPushButton:pressed{background:#dcecff;} QPushButton:disabled{color:#a5b2c2;background:#f5f7fa;}
        QPushButton#primary{color:white;background:#287bf0;border:1px solid #287bf0;font-weight:600;} QPushButton#primary:hover{background:#176ce0;}
        QPushButton#primary:disabled{background:#a5c6f2;border-color:#a5c6f2;} QPushButton#secondary{background:#f7faff;color:#3673c2;}
        QPushButton#viewToggle{padding:5px 12px;background:#f5f8fc;color:#7c91ad;border-color:#e0e8f3;} QPushButton#viewToggle:checked{background:#e1eeff;color:#226ed6;border-color:#90b8ee;font-weight:600;}
        QPushButton#headerImport{background:#233d5d;color:#d5e5ff;border-color:#3a5372;} QPushButton#tool{border-color:transparent;padding:6px 9px;}
        QLabel#sectionTitle{color:#203b5c;font-size:13px;font-weight:700;} QLabel#fieldLabel{color:#8492a5;font-size:10px;}
        QLabel#subtle{color:#8a9ab0;font-size:10px;line-height:1.5;} QLabel#statValue{font-size:21px;font-weight:700;color:#284a74;}
        QFrame#statsCard{background:#f5f8fc;border:1px solid #e6edf5;border-radius:8px;} QFrame#statsCard QLabel{background:transparent;}
        QLineEdit,QDoubleSpinBox,QComboBox{border:1px solid #dce5f0;background:#fbfcfe;border-radius:5px;padding:7px;color:#304b6e;min-height:17px;font-size:11px;}
        QLineEdit:focus,QDoubleSpinBox:focus,QComboBox:focus{border:1px solid #71a7ef;background:white;} QComboBox::drop-down{border:0;width:18px;}
        QDoubleSpinBox:disabled{background:#f4f6f9;color:#acb7c5;} QCheckBox{font-size:10px;spacing:6px;background:transparent;color:#657d99;}
        QCheckBox::indicator{width:13px;height:13px;border:1px solid #bdcde0;border-radius:3px;background:white;}QCheckBox::indicator:checked{background:#2e80ef;border-color:#2e80ef;}
        QFrame#divider{color:#e7edf5;max-height:1px;} QLabel#localNote{background:#f0f8f6;color:#649486;font-size:10px;padding:12px;border-radius:7px;}
        QLabel#eyebrow{color:#94a7be;background:transparent;font-size:9px;padding-left:8px;letter-spacing:2px;}
        QLabel#notice{background:#e8f1fd;color:#6283af;padding:9px 12px;border-radius:6px;font-size:10px;}
        QFrame#canvasFrame{border:1px solid #d4dfed;border-radius:6px;background:#f8fbfe;} QLabel#workflow{font-size:10px;color:#8b9db2;line-height:1.8;padding:12px 4px;}
        QListWidget#orderList{border:0;background:transparent;outline:0;} QListWidget#orderList::item{background:#ffffff;border:1px solid #dfe8f3;border-radius:6px;color:#6a83a1;padding:5px;font-size:10px;}
        QListWidget#orderList::item:selected{background:#e9f2ff;border:1px solid #87b2ec;color:#2c73cc;}
        QStatusBar{background:#ffffff;border-top:1px solid #dfe7f1;color:#8c9bb0;font-size:10px;}QStatusBar::item{border:0;}QPushButton#zoomButton{padding:1px 8px;border:0;}
        QSlider::groove:horizontal{height:4px;background:#e1e9f2;border-radius:2px;}QSlider::sub-page:horizontal{background:#71a7f0;border-radius:2px;}QSlider::handle:horizontal{background:#287bf0;border-radius:6px;width:12px;margin:-4px 0;}
        QScrollBar:horizontal{height:7px;background:#e9eef5;border:0;}QScrollBar:vertical{width:7px;background:#eff3f8;border:0;}QScrollBar::handle{background:#bfcede;border-radius:3px;min-width:22px;min-height:22px;}QScrollBar::add-line,QScrollBar::sub-line{width:0;height:0;}
        QSplitter::handle{background:#dfe7f1;} QToolTip{background:#172e4d;color:#eef6ff;border:0;padding:5px;}
    )");
    ot::StudioWindow window;
    window.show();
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
