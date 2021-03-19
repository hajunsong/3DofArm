#include "Logging/logger.h"

Logger::Logger(QObject *parent, QString fileName, QPlainTextEdit *editor) : QObject(parent) {
    m_editor = editor;
    m_showDate = true;
    if (!fileName.isEmpty()) {
        file = new QFile;
        file->setFileName(fileName);
        file->open(QIODevice::Append | QIODevice::Text);
    }
}

void Logger::write(const QString &value) {
    QString text = value;// + "";
    if (m_showDate)
        text = QDateTime::currentDateTime().toString("ss.zzz,") + text + "\n";
    QTextStream out(file);
//    out.setCodec("UTF-8");
    if (file != nullptr) {
        out << text;
    }
    if (m_editor != nullptr)
        m_editor->appendPlainText(text);
}

void Logger::setShowDateTime(bool value) {
    m_showDate = value;
}

Logger::~Logger() {
    if (file != nullptr)
        file->close();
}
