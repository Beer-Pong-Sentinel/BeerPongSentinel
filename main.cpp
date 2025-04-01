#include <Spinnaker.h>
#include <opencv2/opencv.hpp>

#include "MainWindow.h"

#include <QApplication>
#include <QLocale>
#include <QTranslator>
#include <QIcon>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setWindowIcon(QIcon(":/icon.png")); // Use the correct path to your icon

    QTranslator translator;
    const QStringList uiLanguages = QLocale::system().uiLanguages();
    for (const QString &locale : uiLanguages) {
        const QString baseName = "BeerPongSentinel_" + QLocale(locale).name();
        if (translator.load(":/i18n/" + baseName)) {
            a.installTranslator(&translator);
            break;
        }
    }
    MainWindow w;
    w.show();
    QIcon icon(":/icon.png");
    w.setWindowIcon(QIcon(":/icon.png")); // Use the correct path to your icon

    return a.exec();
}
