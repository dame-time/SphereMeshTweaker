#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

class OpenGLWidget;

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onCheckSMFilled(int state);
    void onCheckSMVisible(int state);

    void onSlider1ValueChanged(int value);
    void onLabelEditingFinished();

    void onSlider2ValueChanged(int value);
    void onSizeLabelEditingFinished();

private:
    Ui::MainWindow *ui;
    OpenGLWidget *openGLWidget;

    void onSaveMesh();
    void onLoadMesh();

    void onDuplicateSphere();
    void onLinkSpheres();
    void onUnlinkSpheres();
};
#endif // MAINWINDOW_H