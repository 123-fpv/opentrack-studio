#include "imageimport.h"

#include <QCoreApplication>
#include <QDataStream>
#include <QDir>
#include <QFile>
#include <QImage>
#include <QMimeData>
#include <QTemporaryDir>
#include <QUrl>
#include <iostream>

namespace {
int failures = 0;
void require(bool condition, const char *description) {
    if (!condition) {
        std::cerr << "FAIL: " << description << '\n';
        ++failures;
    }
}

bool write(const QString &path, const QByteArray &bytes) {
    QFile file(path);
    return file.open(QIODevice::WriteOnly) && file.write(bytes) == bytes.size();
}

void rejected(const QList<QUrl> &urls, const char *description) {
    QMimeData mime;
    mime.setUrls(urls);
    const auto result = ot::inspectImageDrop(&mime);
    require(!result.supported && result.paths.isEmpty() && !result.error.isEmpty(), description);
}

// A BMP header can advertise dimensions without allocating an enormous raster.
// The helper must inspect metadata and reject this before a full image decode.
QByteArray bmpHeader(qint32 width, qint32 height) {
    QByteArray bytes;
    QDataStream stream(&bytes, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.writeRawData("BM", 2);
    stream << quint32(54) << quint16(0) << quint16(0) << quint32(54);
    stream << quint32(40) << width << height << quint16(1) << quint16(24);
    stream << quint32(0) << quint32(0) << qint32(0) << qint32(0) << quint32(0) << quint32(0);
    return bytes;
}
} // namespace

int main(int argc, char **argv) {
    QCoreApplication application(argc, argv);
    QTemporaryDir temporary;
    require(temporary.isValid(), "temporary directory");
    if (!temporary.isValid())
        return 1;
    QImage image(24, 16, QImage::Format_RGB32);
    image.fill(Qt::white);
    const QString png = temporary.filePath(QStringLiteral("赛道 图片 #1.PnG"));
    const QString jpg = temporary.filePath(QStringLiteral("camera angle.JPEG"));
    const QString bmp = temporary.filePath(QStringLiteral("plan.BMP"));
    require(image.save(png, "PNG") && image.save(jpg, "JPEG") && image.save(bmp, "BMP"),
            "PNG JPEG BMP test images can be written");

    QMimeData mime;
    mime.setUrls({QUrl::fromLocalFile(png)});
    auto result = ot::inspectImageDrop(&mime);
    require(result.supported && result.error.isEmpty() && result.paths == QStringList{png},
            "Unicode spaces hash and mixed-case suffix survive file URL decoding");
    require(ot::localImagePaths(&mime) == result.paths && ot::imageDropError(&mime).isEmpty(),
            "convenience APIs retain all-or-nothing semantics");
    mime.setUrls({QUrl::fromLocalFile(jpg), QUrl::fromLocalFile(bmp), QUrl::fromLocalFile(png)});
    result = ot::inspectImageDrop(&mime);
    require(result.supported && result.paths == QStringList{jpg, bmp, png},
            "multiple local image formats retain input order");
    QList<QUrl> eight;
    for (int i = 0; i < 8; ++i)
        eight.append(QUrl::fromLocalFile(png));
    mime.setUrls(eight);
    require(ot::inspectImageDrop(&mime).paths.size() == 8,
            "eight explicit inputs are accepted without silent deduplication");
    eight.append(QUrl::fromLocalFile(png));
    rejected(eight, "nine inputs rejected as a whole");
    rejected({QUrl(QStringLiteral("https://example.invalid/track.png"))},
             "remote URL is never fetched");
    rejected({QUrl::fromLocalFile(png), QUrl(QStringLiteral("https://example.invalid/track.png"))},
             "valid image plus remote URL never returns partial success");
    rejected({QUrl(QStringLiteral("file://remote-server/share/track.png"))},
             "network-share URL rejected before filesystem probing");
    rejected({QUrl::fromLocalFile(temporary.path())}, "directory rejected");
    rejected({QUrl::fromLocalFile(temporary.filePath(QStringLiteral("missing.png")))},
             "missing file rejected");
    const QString text = temporary.filePath(QStringLiteral("notes.txt"));
    require(write(text, "not an image"), "write text fixture");
    rejected({QUrl::fromLocalFile(png), QUrl::fromLocalFile(text)},
             "mixed non-image file rejected without partial path list");
    const QString invalid = temporary.filePath(QStringLiteral("corrupt.jpg"));
    require(write(invalid, "this is not JPEG data"), "write invalid image fixture");
    rejected({QUrl::fromLocalFile(invalid)}, "image suffix with invalid metadata rejected");
    const QString disguised = temporary.filePath(QStringLiteral("renamed-format.png"));
    QByteArray ppm("P6\n1 1\n255\n");
    ppm.append(char(-1));
    ppm.append(char(0));
    ppm.append(char(0));
    require(write(disguised, ppm), "write readable but unsupported format with PNG suffix");
    rejected({QUrl::fromLocalFile(disguised)}, "content format must also be PNG JPEG or BMP");
    const QString empty = temporary.filePath(QStringLiteral("empty.png"));
    require(write(empty, {}), "write empty file");
    rejected({QUrl::fromLocalFile(empty)}, "empty image rejected");
    QMimeData plain;
    plain.setText(png);
    require(!ot::inspectImageDrop(&plain).supported && !ot::inspectImageDrop(nullptr).supported,
            "plain text and null MIME data do not become implicit file imports");

    const QString tooLarge = temporary.filePath(QStringLiteral("large.png"));
    require(image.save(tooLarge, "PNG"), "write byte-limit fixture");
    QFile bytes(tooLarge);
    require(bytes.open(QIODevice::ReadWrite) && bytes.resize(80LL * 1024 * 1024 + 1),
            "create sparse over-limit file");
    bytes.close();
    mime.setUrls({QUrl::fromLocalFile(tooLarge)});
    require(ot::inspectImageDrop(&mime).error.contains(QStringLiteral("80 MiB")),
            "80MiB byte limit checked before image decode");
    require(bytes.open(QIODevice::ReadWrite) && bytes.resize(80LL * 1024 * 1024),
            "resize byte fixture to exact limit");
    bytes.close();
    require(ot::inspectImageDrop(&mime).supported, "exactly80MiB readable metadata is allowed");
    const QString tooManyPixels = temporary.filePath(QStringLiteral("huge.bmp"));
    require(write(tooManyPixels, bmpHeader(10000, 10000)), "write huge-dimension BMP metadata");
    mime.setUrls({QUrl::fromLocalFile(tooManyPixels)});
    result = ot::inspectImageDrop(&mime);
    require(!result.supported && result.paths.isEmpty() &&
                result.error.contains(QStringLiteral("像素")),
            "metadata pixel limit rejected without allocating 100million pixel image");
    require(write(tooManyPixels, bmpHeader(10000, 8000)), "write exact pixel-limit metadata");
    require(ot::inspectImageDrop(&mime).supported,
            "exactly80million pixels allowed without full decode");
    if (!failures)
        std::cout << "PASS: local image drops, Unicode URLs, multiple files, atomic rejection and "
                     "metadata limits.\n";
    return failures ? 1 : 0;
}
