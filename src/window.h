#pragma once
#include "canvas.h"
#include <QMainWindow>
#include <QUndoStack>
#include <functional>
class QLineEdit;
class QLabel;
class QDoubleSpinBox;
class QListWidget;
class QComboBox;
class QSlider;
class QCheckBox;
class QPushButton;
class QStackedWidget;
class QTimer;
namespace ot {
class Scene3D;
class StudioWindow : public QMainWindow {
  public:
    StudioWindow();
    ~StudioWindow() override;
    void applyDocument(const TrackDocument &doc);
    bool runSmokeTest(const QString &directory);
    bool openPath(const QString &path);

  protected:
    void closeEvent(QCloseEvent *) override;
    bool eventFilter(QObject *, QEvent *) override;

  private:
    TrackDocument doc_, gestureBefore_;
    QUndoStack undo_;
    TrackCanvas *canvas_ = nullptr;
    Scene3D *scene3d_ = nullptr;
    QStackedWidget *views_ = nullptr;
    QWidget *properties_ = nullptr;
    QLineEdit *name_ = nullptr;
    QComboBox *scenery_ = nullptr;
    QComboBox *passSide_ = nullptr;
    QComboBox *color_ = nullptr;
    QComboBox *passColumn_ = nullptr, *passLevel_ = nullptr;
    QWidget *passCell_ = nullptr;
    QLabel *passSideLabel_ = nullptr;
    QLabel *widthLabel_ = nullptr, *heightLabel_ = nullptr, *dimensions_ = nullptr,
           *colorMapping_ = nullptr;
    QLabel *gateCount_ = nullptr, *length_ = nullptr, *zoom_ = nullptr, *selectionTitle_ = nullptr,
           *imageStatus_ = nullptr, *notice_ = nullptr;
    QListWidget *order_ = nullptr;
    QDoubleSpinBox *x_ = nullptr, *z_ = nullptr, *alt_ = nullptr, *yaw_ = nullptr,
                   *width_ = nullptr, *height_ = nullptr, *scale_ = nullptr;
    QSlider *opacity_ = nullptr;
    QCheckBox *autoScale_ = nullptr, *imageVisible_ = nullptr;
    QPushButton *recognizeButton_ = nullptr;
    QPushButton *dropZone_ = nullptr;
    QTimer *recognitionTimer_ = nullptr;
    qint64 recognitionStartedMs_ = 0;
    quint64 recognitionSerial_ = 0;
    QString path_, selected_;
    bool updating_ = false, recognizing_ = false;
    quint64 revision_ = 0;
    void setupUi();
    void setupShortcuts();
    void fitViews();
    void setPlacement(const QString &type, double width = 2.4, double height = 2.4);
    void zoomActive(double factor);
    void edit(const QString &title, const std::function<void(TrackDocument &)> &change);
    void pushEdit(const QString &title, const TrackDocument &before, const TrackDocument &after);
    void updateStats();
    void updateSelection(const QString &id);
    void editSelection();
    void editColor(const QColor &color);
    void addGate(const QString &type, QPointF pos, double w, double h);
    void duplicateGate();
    void deleteGate();
    void rotateGate(double delta);
    void moveOrder(int delta);
    void importImage();
    void importImagePath(const QString &path);
    void importPhotosWithAi(const QStringList &paths = {});
    void runImageRecognition(const QString &imagePath, const QImage &image);
    void startRecognition(bool confirmReplace = true);
    void newProject(bool demo = false);
    void openProject();
    bool save(bool as = false);
    void exportTrack();
    void exportPng();
    bool maybeSave();
    void fail(const QString &message);
};
QImage makeExampleImage();
} // namespace ot
