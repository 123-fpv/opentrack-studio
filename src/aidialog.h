#pragma once
#include "aivision.h"
#include <QDialog>
#include <atomic>
#include <memory>
class QListWidget;
class QLineEdit;
class QDoubleSpinBox;
class QPlainTextEdit;
class QCheckBox;
class QLabel;
class QPushButton;
class QProgressBar;
class QStackedWidget;
namespace ot {
class Scene3D;
class AiVisionDialog final : public QDialog {
  public:
    explicit AiVisionDialog(QWidget *parent = nullptr);
    ~AiVisionDialog() override;
    const AiResult &result() const {
        return result_;
    }
    void setImagePaths(const QStringList &paths);

  protected:
    void reject() override;

  private:
    QListWidget *images_ = nullptr;
    QLineEdit *key_ = nullptr;
    QDoubleSpinBox *width_ = nullptr;
    QPlainTextEdit *notes_ = nullptr;
    QCheckBox *remember_ = nullptr;
    QLabel *state_ = nullptr;
    QProgressBar *progress_ = nullptr;
    QPushButton *generate_ = nullptr, *test_ = nullptr, *apply_ = nullptr, *add_ = nullptr,
                *remove_ = nullptr, *forget_ = nullptr;
    QStackedWidget *previewStack_ = nullptr;
    Scene3D *preview_ = nullptr;
    AiResult result_;
    bool busy_ = false;
    quint64 nextThumbnailId_ = 0;
    std::shared_ptr<std::atomic_bool> cancel_;
    void addPhotos();
    AiSettings settings() const;
    bool prepareKey();
    void invalidateDraft(const QString &message = QStringLiteral("输入已更改，请重新生成草稿。"));
    void setBusy(bool busy, const QString &message);
    void testConnection();
    void generateDraft();
};
} // namespace ot
