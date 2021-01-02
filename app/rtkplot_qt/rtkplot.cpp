#include "plotmain.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Plot* w = new Plot();
    //Plot w(nullptr);
    w->show();

    return a.exec();
}
