#pragma once

#include <QCoreApplication>
#include <QFile>
#include <QHash>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QSettings>
#include <QStringList>
#include <QVector>
#include <algorithm>
#include <atomic>

namespace ot::i18n {
// Source strings are stable Chinese catalog keys. Project text and JSON field
// names are never translated. Formatted diagnostics are translated only at UI
// boundaries against complete, cataloged message templates.
inline std::atomic_bool &englishStorage() {
    static std::atomic_bool value{false};
    return value;
}
inline bool english() {
    return englishStorage().load(std::memory_order_relaxed);
}
inline QString language() {
    return english() ? QStringLiteral("en") : QStringLiteral("zh");
}
inline void setLanguage(const QString &code, bool persist = false) {
    const bool en = code.startsWith(QStringLiteral("en"), Qt::CaseInsensitive);
    englishStorage().store(en, std::memory_order_relaxed);
    QCoreApplication::setAttribute(Qt::AA_DontUseNativeDialogs, en);
    if (persist)
        QSettings().setValue(QStringLiteral("interface/language"), en ? "en" : "zh");
}
inline void initialize() {
    setLanguage(QSettings().value(QStringLiteral("interface/language"), "zh").toString());
}
inline const QHash<QString, QString> &catalog() {
    static const QHash<QString, QString> values = [] {
        QFile file(QStringLiteral(":/i18n/en.json"));
        if (!file.open(QIODevice::ReadOnly))
            return QHash<QString, QString>();
        const auto object = QJsonDocument::fromJson(file.readAll()).object();
        QHash<QString, QString> result;
        for (auto entry = object.begin(); entry != object.end(); ++entry)
            result.insert(entry.key(), entry.value().toString());
        return result;
    }();
    return values;
}
inline QString text(const QString &source) {
    if (!english() || source.isEmpty())
        return source;
    const auto &values = catalog();
    const auto exact = values.constFind(source);
    if (exact != values.cend())
        return exact.value();
    static const QRegularExpression han(QStringLiteral("[\\x{3400}-\\x{9fff}]"));
    if (!source.contains(han))
        return source;
    struct Format {
        QRegularExpression expression;
        QString target;
        qsizetype literalLength = 0;
    };
    static const QVector<Format> formats = [&values] {
        QVector<Format> result;
        const QRegularExpression placeholder(QStringLiteral("%([1-9][0-9]?)"));
        for (auto entry = values.cbegin(); entry != values.cend(); ++entry) {
            auto matches = placeholder.globalMatch(entry.key());
            if (!matches.hasNext())
                continue;
            QString pattern = QStringLiteral("\\A"), target = entry.value();
            qsizetype previous = 0;
            QHash<QString, int> captures;
            int count = 0;
            while (matches.hasNext()) {
                const auto match = matches.next();
                pattern += QRegularExpression::escape(
                    entry.key().mid(previous, match.capturedStart() - previous));
                const QString number = match.captured(1);
                if (captures.contains(number))
                    pattern += QStringLiteral("\\g{%1}").arg(captures.value(number));
                else {
                    captures.insert(number, ++count);
                    pattern += QStringLiteral("(?<p%1>.*?)").arg(number);
                }
                previous = match.capturedEnd();
            }
            pattern +=
                QRegularExpression::escape(entry.key().mid(previous)) + QStringLiteral("\\z");
            QString literals = entry.key();
            literals.remove(placeholder);
            result.append(
                {QRegularExpression(pattern, QRegularExpression::DotMatchesEverythingOption),
                 target, literals.size()});
        }
        std::sort(result.begin(), result.end(), [](const Format &a, const Format &b) {
            if (a.literalLength != b.literalLength)
                return a.literalLength > b.literalLength;
            return a.expression.pattern() < b.expression.pattern();
        });
        return result;
    }();
    for (const auto &format : formats) {
        const auto match = format.expression.match(source);
        if (!match.hasMatch())
            continue;
        QString result;
        qsizetype previous = 0;
        static const QRegularExpression marker(QStringLiteral("%([1-9][0-9]?)"));
        auto substitutions = marker.globalMatch(format.target);
        while (substitutions.hasNext()) {
            const auto substitution = substitutions.next();
            result += format.target.mid(previous, substitution.capturedStart() - previous);
            const QString name = QStringLiteral("p") + substitution.captured(1);
            result += match.hasCaptured(name) ? match.captured(name) : substitution.captured();
            previous = substitution.capturedEnd();
        }
        result += format.target.mid(previous);
        return result;
    }
    return source;
}
inline QStringList messages(const QStringList &source) {
    QStringList result;
    for (const auto &message : source)
        result.append(text(message));
    return result;
}
} // namespace ot::i18n
