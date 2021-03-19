#include "MainWindow/mainwindow.h"
#include "ui_mainwindow.h"
#include "customsettings.h"

CustomSettings::CustomSettings(void *_ui)
{
    ui = static_cast<Ui::MainWindow*>(_ui);
}

void CustomSettings::saveConfigFile()
{
    QSettings settings(configFile(), QSettings::IniFormat);

//    settings.setValue("geometry", saveGeometry());
//    settings.setValue("windowState", saveState());

    QString port = static_cast<Ui::MainWindow*>(ui)->txtPort->text();
    int baud = static_cast<Ui::MainWindow*>(ui)->cbBaud->currentIndex();

    settings.setValue("PORT", port);
    settings.setValue("BAUD", baud);

    settings.sync();
}

void CustomSettings::loadConfigFile()
{
    if (!QFile::exists(configFile())) return;

    QSettings settings(configFile(), QSettings::IniFormat);

//    restoreGeometry(settings.value("geometry").toByteArray());
//    restoreState(settings.value("windowState").toByteArray());

    QString port = settings.value("PORT").toString();
    int baud = settings.value("BAUD").toInt();

    static_cast<Ui::MainWindow*>(ui)->txtPort->setText(port);
    static_cast<Ui::MainWindow*>(ui)->cbBaud->setCurrentIndex(baud);
}

QString CustomSettings::configFile()
{
    QString filePath = qApp->applicationDirPath() + "/config.ini";
//    qDebug() << filePath;
    return filePath;
}

