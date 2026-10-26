#include "imageimport.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QImageReader>
#include <QMimeData>
#include <QSet>
#include <QUrl>

namespace ot {
namespace {
constexpr int MaximumImages = 8;
constexpr qint64 MaximumBytes = 80LL * 1024 * 1024;
constexpr qint64 MaximumPixels = 80LL * 1000 * 1000;

ImageDropResult rejected(const QString &message) {
    return {{}, message, false};
}

bool supportedSuffix(const QString &suffix) {
    static const QSet<QString> allowed{QStringLiteral("png"), QStringLiteral("jpg"),
                                       QStringLiteral("jpeg"), QStringLiteral("bmp")};
    return allowed.contains(suffix.toLower());
}

bool supportedFormat(const QByteArray &format) {
    const auto lower = format.toLower();
    return lower == "png" || lower == "jpeg" || lower == "jpg" || lower == "bmp";
}
} // namespace

ImageDropResult inspectImageDrop(const QMimeData *mimeData) {
    if (!mimeData || !mimeData->hasUrls())
        return rejected(QStringLiteral("请拖入本地 PNG、JPG、JPEG 或 BMP 图片文件。"));
    const auto urls = mimeData->urls();
    if (urls.isEmpty() || urls.size() > MaximumImages)
        return rejected(QStringLiteral("一次可导入 1 至 8 张本地图片，请减少后重试。"));

    QStringList paths;
    paths.reserve(urls.size());
    // Validate the whole URL list before probing any image reader. A remote or
    // mixed drop is rejected as a whole, with no URL handed to a network API.
    for (QUrl url : urls) {
        if (!url.isValid() || !url.isLocalFile() || url.hasQuery() || url.hasFragment())
            return rejected(
                QStringLiteral("只支持本地图片文件，不支持网页、远程地址或带参数的文件链接。"));
        if (!url.host().isEmpty()) {
            if (url.host().compare(QStringLiteral("localhost"), Qt::CaseInsensitive) != 0)
                return rejected(QStringLiteral("请先把网络图片保存到本机，再拖入文件。"));
            url.setHost(QString());
        }
        const QString path = QDir::fromNativeSeparators(url.toLocalFile());
        if (path.isEmpty() || path.startsWith(QStringLiteral("//")))
            return rejected(QStringLiteral("不支持网络共享地址，请拖入本机图片文件。"));
        const QFileInfo info(path);
        if (!info.exists() || !info.isFile())
            return rejected(QStringLiteral("“%1”不是现有的图片文件，不能导入目录或已删除文件。")
                                .arg(info.fileName()));
        if (!supportedSuffix(info.suffix()))
            return rejected(QStringLiteral("“%1”的文件类型不受支持；请使用 PNG、JPG、JPEG 或 BMP。")
                                .arg(info.fileName()));
        if (info.size() <= 0)
            return rejected(
                QStringLiteral("“%1”是空文件，请换一张有效图片。").arg(info.fileName()));
        if (info.size() > MaximumBytes)
            return rejected(
                QStringLiteral("“%1”超过 80 MiB，请压缩或裁剪后导入。").arg(info.fileName()));
        paths.append(QDir::cleanPath(info.absoluteFilePath()));
    }

    for (const QString &path : paths) {
        QFile source(path);
        if (!source.open(QIODevice::ReadOnly))
            return rejected(QStringLiteral("无法读取“%1”，请检查文件权限或占用情况。")
                                .arg(QFileInfo(path).fileName()));
        QImageReader reader(&source);
        reader.setDecideFormatFromContent(true);
        if (!reader.canRead() || !supportedFormat(reader.format()))
            return rejected(QStringLiteral("“%1”不是可读取的 PNG、JPEG 或 BMP 图片，文件可能损坏。")
                                .arg(QFileInfo(path).fileName()));
        const QSize size = reader.size();
        if (!size.isValid() || size.isEmpty())
            return rejected(QStringLiteral("无法读取“%1”的图片尺寸，文件可能损坏。")
                                .arg(QFileInfo(path).fileName()));
        if (qint64(size.width()) * size.height() > MaximumPixels)
            return rejected(QStringLiteral("“%1”超过 8000 万像素，请缩小图片后导入。")
                                .arg(QFileInfo(path).fileName()));
    }
    return {paths, {}, true};
}

QStringList localImagePaths(const QMimeData *mimeData) {
    return inspectImageDrop(mimeData).paths;
}

QString imageDropError(const QMimeData *mimeData) {
    return inspectImageDrop(mimeData).error;
}
} // namespace ot
