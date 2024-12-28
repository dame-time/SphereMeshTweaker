#include "mainwindow.h"

#include <QApplication>
#include <QSurfaceFormat>
#include <QLocale>
#include <QTranslator>
#include <QCoreApplication>
#include <QDir>

int main(int argc, char *argv[])
{
    QDir::setCurrent(QCoreApplication::applicationDirPath());
    QApplication a(argc, argv);

    QTranslator translator;
    const QStringList uiLanguages = QLocale::system().uiLanguages();
    for (const QString &locale : uiLanguages) {
        const QString baseName = "SphereMeshTweaker_" + QLocale(locale).name();
        if (translator.load(":/i18n/" + baseName)) {
            a.installTranslator(&translator);
            break;
        }
    }
    MainWindow w;
    w.show();
    return a.exec();
}
