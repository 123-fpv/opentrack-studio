#include "aidialog.h"
#include "scene3d.h"
#include <QFutureWatcher>
#include <QImageReader>
#include <QThreadPool>
#include <QtConcurrentRun>
#include <QtWidgets>

namespace ot {
namespace {
QPushButton *action(const QString &text, const QString &style = {}) {
    auto *b = new QPushButton(text);
    b->setCursor(Qt::PointingHandCursor);
    if (!style.isEmpty())
        b->setObjectName(style);
    return b;
}
QLabel *text(const QString &s, const QString &style = {}) {
    auto *l = new QLabel(s);
    l->setTextFormat(Qt::PlainText);
    l->setWordWrap(true);
    if (!style.isEmpty())
        l->setObjectName(style);
    return l;
}
struct WorkResult {
    AiResult result;
    QString loadError;
};
QThreadPool *thumbnailPool() {
    static QThreadPool *pool = [] {
        auto *value = new QThreadPool(QCoreApplication::instance());
        value->setMaxThreadCount(2);
        value->setExpiryTimeout(10000);
        return value;
    }();
    return pool;
}
QString warningTooltip(const QStringList &warnings) {
    return "<pre style=\"white-space:pre-wrap\">" + warnings.join('\n').toHtmlEscaped() + "</pre>";
}
} // namespace
AiVisionDialog::AiVisionDialog(QWidget *parent) : QDialog(parent) {
    setWindowTitle("照片生成赛道 · DeepSeek V4.1 Flash");
    setWindowIcon(QIcon(":/icons/app.png"));
    resize(960, 720);
    setMinimumSize(820, 620);
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(22, 20, 22, 18);
    layout->setSpacing(13);
    layout->addWidget(text("实拍 / 航拍照片 → 可编辑的 3D 赛道草稿", "sectionTitle"));
    layout->addWidget(text(
        "选择同一飞场的 1–8 "
        "张照片，尽量覆盖全部门架并包含重叠视角。已知门宽用于估算比例；距离、高度和门序仍需校对。",
        "subtle"));
    auto *body = new QHBoxLayout;
    body->setSpacing(18);
    auto *left = new QVBoxLayout;
    left->addWidget(text("参考照片", "fieldLabel"));
    images_ = new QListWidget;
    images_->setIconSize(QSize(110, 78));
    images_->setSpacing(6);
    images_->setMinimumWidth(265);
    images_->setMaximumWidth(320);
    images_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    left->addWidget(images_, 1);
    auto *photoButtons = new QHBoxLayout;
    add_ = action("＋ 添加照片", "secondary");
    remove_ = action("移除", "tool");
    photoButtons->addWidget(add_, 1);
    photoButtons->addWidget(remove_);
    left->addLayout(photoButtons);
    body->addLayout(left, 0);
    connect(add_, &QPushButton::clicked, this, [this] { addPhotos(); });
    connect(remove_, &QPushButton::clicked, this, [this] {
        const auto selected = images_->selectedItems();
        if (selected.isEmpty())
            return;
        qDeleteAll(selected);
        invalidateDraft();
    });
    auto *right = new QVBoxLayout;
    right->setSpacing(10);
    auto *provider = text("DeepSeek 官方 · V4.1 Flash · 支持多图输入", "notice");
    right->addWidget(provider);
    auto *form = new QFormLayout;
    form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    form->setSpacing(9);
    key_ = new QLineEdit;
    key_->setEchoMode(QLineEdit::Password);
    key_->setPlaceholderText("粘贴 DeepSeek API Key");
    key_->setMaxLength(2000);
    key_->setText(loadApiKey());
    form->addRow("API 密钥", key_);
    width_ = new QDoubleSpinBox;
    width_->setRange(.2, 20);
    width_->setValue(2.4);
    width_->setSingleStep(.1);
    width_->setSuffix(" m");
    form->addRow("已知门宽", width_);
    right->addLayout(form);
    auto *credentialRow = new QHBoxLayout;
    remember_ = new QCheckBox("在本机记住密钥");
    remember_->setChecked(true);
    credentialRow->addWidget(remember_);
    credentialRow->addStretch();
    forget_ = action("清除本机密钥", "tool");
    credentialRow->addWidget(forget_);
    right->addLayout(credentialRow);
    connect(forget_, &QPushButton::clicked, this, [this] {
        if (busy_)
            return;
        clearApiKey();
        key_->clear();
        state_->setText("已清除本机保存的密钥。");
    });
    right->addWidget(text("补充说明（可选，最多 4000 字）", "fieldLabel"));
    notes_ = new QPlainTextEdit;
    notes_->setPlaceholderText("例如：起点是左下角绿门，顺时针飞行，门高约 2.4 "
                               "米；中间是上下穿越。请只填写你确认的信息。");
    notes_->setMaximumHeight(68);
    right->addWidget(notes_);
    previewStack_ = new QStackedWidget;
    auto *placeholder =
        text("生成后在这里检查 3D 草稿\n\n未看到的门架和无法确定的尺度会提示校对。", "notice");
    placeholder->setAlignment(Qt::AlignCenter);
    previewStack_->addWidget(placeholder);
    if (QGuiApplication::platformName() != "offscreen") {
        preview_ = new Scene3D;
        previewStack_->addWidget(preview_);
    }
    previewStack_->setMinimumHeight(190);
    right->addWidget(previewStack_, 1);
    body->addLayout(right, 1);
    layout->addLayout(body, 1);
    state_ = text("点击“生成草稿”时，会把所选照片的缩小副本发送到 DeepSeek 官方 API。", "subtle");
    state_->setMinimumHeight(30);
    layout->addWidget(state_);
    progress_ = new QProgressBar;
    progress_->setRange(0, 0);
    progress_->setFixedHeight(4);
    progress_->hide();
    layout->addWidget(progress_);
    auto *buttons = new QHBoxLayout;
    auto *close = action("取消", "tool");
    buttons->addWidget(close);
    buttons->addStretch();
    test_ = action("测试连接", "secondary");
    generate_ = action("生成草稿", "primary");
    apply_ = action("使用这份草稿", "secondary");
    apply_->setEnabled(false);
    buttons->addWidget(test_);
    buttons->addWidget(generate_);
    buttons->addWidget(apply_);
    layout->addLayout(buttons);
    connect(close, &QPushButton::clicked, this, [this] { reject(); });
    connect(test_, &QPushButton::clicked, this, [this] { testConnection(); });
    connect(generate_, &QPushButton::clicked, this, [this] { generateDraft(); });
    connect(apply_, &QPushButton::clicked, this, [this] {
        if (result_.ok && !busy_)
            accept();
    });
    connect(width_, &QDoubleSpinBox::valueChanged, this, [this](double) { invalidateDraft(); });
    connect(notes_, &QPlainTextEdit::textChanged, this, [this] {
        const QString content = notes_->toPlainText();
        if (content.size() > 4000) {
            int limit = 4000;
            if (content[limit - 1].isHighSurrogate() && content[limit].isLowSurrogate())
                --limit;
            const int position = std::min(notes_->textCursor().position(), limit);
            const QSignalBlocker blocked(notes_);
            notes_->setPlainText(content.left(limit));
            auto cursor = notes_->textCursor();
            cursor.setPosition(position);
            notes_->setTextCursor(cursor);
        }
        invalidateDraft();
    });
}
AiVisionDialog::~AiVisionDialog() {
    if (cancel_)
        cancel_->store(true);
}
void AiVisionDialog::reject() {
    if (cancel_)
        cancel_->store(true);
    QDialog::reject();
}
AiSettings AiVisionDialog::settings() const {
    AiSettings s;
    s.apiKey = key_->text().trimmed();
    s.knownGateWidth = width_->value();
    s.notes = notes_->toPlainText().left(4000);
    return s;
}
bool AiVisionDialog::prepareKey() {
    if (key_->text().trimmed().isEmpty()) {
        state_->setText("请先填写 API 密钥。");
        key_->setFocus();
        return false;
    }
    if (remember_->isChecked()) {
        QString error;
        if (!storeApiKey(key_->text().trimmed(), &error)) {
            state_->setText(error);
            return false;
        }
    } else
        clearApiKey();
    return true;
}
void AiVisionDialog::invalidateDraft(const QString &message) {
    if (busy_)
        return;
    result_ = AiResult();
    apply_->setEnabled(false);
    if (preview_)
        preview_->setDocument(nullptr);
    previewStack_->setCurrentIndex(0);
    state_->setToolTip({});
    if (!message.isEmpty())
        state_->setText(message);
}
void AiVisionDialog::setBusy(bool value, const QString &message) {
    busy_ = value;
    state_->setText(message);
    progress_->setVisible(value);
    for (auto *b : {generate_, test_, add_, remove_, forget_})
        b->setEnabled(!value);
    key_->setEnabled(!value);
    width_->setEnabled(!value);
    notes_->setEnabled(!value);
    remember_->setEnabled(!value);
    apply_->setEnabled(!value && result_.ok);
}
void AiVisionDialog::addPhotos() {
    if (busy_)
        return;
    const auto paths = QFileDialog::getOpenFileNames(this, "选择同一飞场的照片", {},
                                                     "照片 (*.jpg *.jpeg *.png *.bmp)");
    if (!paths.isEmpty())
        setImagePaths(paths);
}
void AiVisionDialog::setImagePaths(const QStringList &paths) {
    if (busy_)
        return;
    int added = 0, skipped = 0;
    bool limitReached = false;
    for (const auto &supplied : paths) {
        const QFileInfo info(supplied);
        const QString p = info.canonicalFilePath();
        bool duplicate = false;
        for (int i = 0; i < images_->count(); ++i)
            if (QString::compare(images_->item(i)->data(Qt::UserRole).toString(), p,
                                 Qt::CaseInsensitive) == 0)
                duplicate = true;
        if (duplicate)
            continue;
        if (images_->count() >= 8) {
            limitReached = true;
            break;
        }
        QImageReader reader(p);
        const auto size = reader.size();
        if (p.isEmpty() || !info.isFile() || !size.isValid() ||
            qint64(size.width()) * size.height() > 80000000 || info.size() > 80LL * 1024 * 1024) {
            ++skipped;
            continue;
        }
        const quint64 thumbnailId = ++nextThumbnailId_;
        auto *item =
            new QListWidgetItem(style()->standardIcon(QStyle::SP_FileIcon), info.fileName());
        item->setData(Qt::UserRole, p);
        item->setData(Qt::UserRole + 1, QVariant::fromValue(thumbnailId));
        item->setToolTip(p);
        item->setSizeHint(QSize(235, 87));
        images_->addItem(item);
        ++added;
        auto *watcher = new QFutureWatcher<QImage>(this);
        connect(watcher, &QFutureWatcher<QImage>::finished, this, [this, watcher, p, thumbnailId] {
            const auto thumbnail = watcher->result();
            watcher->deleteLater();
            for (int i = 0; i < images_->count(); ++i) {
                auto *current = images_->item(i);
                if (current->data(Qt::UserRole).toString() != p ||
                    current->data(Qt::UserRole + 1).toULongLong() != thumbnailId)
                    continue;
                if (!thumbnail.isNull())
                    current->setIcon(QIcon(QPixmap::fromImage(thumbnail)));
                else
                    current->setText(QFileInfo(p).fileName() + "\n缩略图不可用");
                break;
            }
        });
        watcher->setFuture(QtConcurrent::run(thumbnailPool(), [p] {
            QImageReader thumbnailReader(p);
            thumbnailReader.setAutoTransform(true);
            const QSize original = thumbnailReader.size();
            if (!original.isValid() || qint64(original.width()) * original.height() > 80000000 ||
                QFileInfo(p).size() > 80LL * 1024 * 1024)
                return QImage();
            thumbnailReader.setScaledSize(original.scaled(180, 120, Qt::KeepAspectRatio));
            return thumbnailReader.read();
        }));
    }
    if (added)
        invalidateDraft(QString("已添加 %1 张照片，请重新生成草稿。").arg(added));
    if (limitReached)
        state_->setText("最多使用 8 张照片，请移除多余照片后再添加。");
    else if (skipped)
        state_->setText(
            QString("已添加 %1 张照片，跳过 %2 张无法读取或过大的照片。").arg(added).arg(skipped));
}
void AiVisionDialog::testConnection() {
    if (busy_ || !prepareKey())
        return;
    cancel_ = std::make_shared<std::atomic_bool>(false);
    const auto cancel = cancel_;
    const auto config = settings();
    auto *watcher = new QFutureWatcher<QString>(this);
    setBusy(true, "正在检查密钥与模型的图片输入能力…");
    connect(watcher, &QFutureWatcher<QString>::finished, this, [this, watcher] {
        const QString error = watcher->result();
        watcher->deleteLater();
        setBusy(false, error.isEmpty() ? "连接成功：DeepSeek V4.1 Flash 支持图片输入。" : error);
    });
    watcher->setFuture(
        QtConcurrent::run([config, cancel] { return validateVisionModel(config, cancel); }));
}
void AiVisionDialog::generateDraft() {
    if (busy_)
        return;
    if (images_->count() == 0) {
        state_->setText("请先添加照片。");
        return;
    }
    if (!prepareKey())
        return;
    QStringList paths;
    for (int i = 0; i < images_->count(); ++i)
        paths << images_->item(i)->data(Qt::UserRole).toString();
    const auto config = settings();
    cancel_ = std::make_shared<std::atomic_bool>(false);
    const auto cancel = cancel_;
    auto *watcher = new QFutureWatcher<WorkResult>(this);
    invalidateDraft({});
    setBusy(true, "正在分析照片并估算门架布局，通常需要几十秒…");
    connect(watcher, &QFutureWatcher<WorkResult>::finished, this, [this, watcher] {
        const auto work = watcher->result();
        watcher->deleteLater();
        result_ = work.result;
        if (!work.loadError.isEmpty()) {
            setBusy(false, work.loadError);
            return;
        }
        if (!result_.ok) {
            setBusy(false, result_.error);
            return;
        }
        if (preview_) {
            preview_->setDocument(&result_.document);
            previewStack_->setCurrentWidget(preview_);
            QTimer::singleShot(0, preview_, [this] { preview_->fitTrack(); });
        }
        setBusy(false, QString("生成 %1 个门架。请检查 3D 草稿；距离、高度、方向和门序需要校对。")
                           .arg(result_.document.gates.size()));
        state_->setToolTip(warningTooltip(result_.warnings));
    });
    watcher->setFuture(QtConcurrent::run([paths, config, cancel] {
        WorkResult work;
        QList<QImage> images;
        for (const auto &path : paths) {
            if (cancel->load()) {
                work.loadError = "已取消。";
                return work;
            }
            QImageReader reader(path);
            reader.setAutoTransform(true);
            const QSize size = reader.size();
            if (!size.isValid() || qint64(size.width()) * size.height() > 80000000) {
                work.loadError = "照片尺寸无效或过大。";
                return work;
            }
            reader.setScaledSize(size.scaled(1600, 1600, Qt::KeepAspectRatio));
            auto image = reader.read();
            if (image.isNull()) {
                work.loadError = "无法读取照片：" + QFileInfo(path).fileName();
                return work;
            }
            images.append(image);
        }
        work.result = generateWithVision(images, config, cancel);
        return work;
    }));
}
} // namespace ot
