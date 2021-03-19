#pragma once

#include <QSettings>
#include <QFile>
#include <QDesktopServices>
#include <QApplication>
#include <QWidget>
#include <QMainWindow>

class CustomSettings : public QSettings{
public:
    CustomSettings(void* _ui);
    void saveConfigFile();
    void loadConfigFile();
    QString configFile();

private:
    void *ui;
};
