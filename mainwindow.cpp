#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "openglwidget.h"

#include <QAction>
#include <QIcon>
#include <QDir>
#include <QCheckBox>
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow), openGLWidget(new OpenGLWidget(this))
{
    ui->setupUi(this);
    setCentralWidget(openGLWidget);

    QString iconPath = QDir::currentPath() + "/icons/";

    ui->topToolBar->setIconSize(QSize(24, 24));
    ui->leftToolBar->setIconSize(QSize(24, 24));

    QAction *leftAction1 = new QAction(QIcon(iconPath + "select.png"), "Select", this);
    QAction *leftAction2 = new QAction(QIcon(iconPath + "plus.png"), "Add", this);
    QAction *leftAction3 = new QAction(QIcon(iconPath + "eraser.png"), "Delete", this);

    connect(leftAction2, &QAction::triggered, this, &MainWindow::onDuplicateSphere);

    ui->leftToolBar->addAction(leftAction1);
    ui->leftToolBar->addAction(leftAction2);
    ui->leftToolBar->addAction(leftAction3);

    QAction *topAction1 = new QAction(QIcon(iconPath + "save.png"), "Save", this);
    QAction *topAction2 = new QAction(QIcon(iconPath + "export.png"), "Export As...", this);

    connect(topAction1, &QAction::triggered, this, &MainWindow::onSaveMesh);
    connect(topAction2, &QAction::triggered, this, &MainWindow::onLoadMesh);

    ui->topToolBar->addAction(topAction1);
    ui->topToolBar->addAction(topAction2);

    ui->horizontalSlider->setMinimum(0);
    ui->spinBox->setMinimum(0);
    ui->horizontalSlider->setMaximum(50);
    ui->spinBox->setMaximum(50);

    ui->horizontalSlider_2->setMinimum(0);
    ui->spinBox_2->setMinimum(0.0);
    ui->horizontalSlider_2->setMaximum(20);
    ui->spinBox_2->setMaximum(1.0);

    connect(ui->checkBox, &QCheckBox::stateChanged, this, &MainWindow::onCheckSMFilled);
    connect(ui->checkBox_2, &QCheckBox::stateChanged, this, &MainWindow::onCheckSMVisible);

    connect(ui->horizontalSlider, &QSlider::valueChanged, this, &MainWindow::onSlider1ValueChanged);
    connect(ui->spinBox, QOverload<int>::of(&QSpinBox::valueChanged), ui->horizontalSlider, &QSlider::setValue);

    connect(ui->horizontalSlider_2, &QSlider::valueChanged, this, &MainWindow::onSlider2ValueChanged);
    connect(ui->spinBox_2, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onSizeLabelEditingFinished);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onCheckSMFilled(int state)
{
    if (state == Qt::Checked)
        openGLWidget->smRenderer->filled = true;
    else
        openGLWidget->smRenderer->filled = false;

    openGLWidget->update();
}

void MainWindow::onCheckSMVisible(int state)
{
    if (state == Qt::Checked)
        openGLWidget->smRenderer->visible = true;
    else
        openGLWidget->smRenderer->visible = false;

    openGLWidget->update();
}

void MainWindow::onSlider1ValueChanged(int value)
{
    openGLWidget->smRenderer->numberOfSpheres = value;
    ui->spinBox->setValue(value);

    openGLWidget->update();
}

void MainWindow::onLabelEditingFinished()
{
    int value = ui->spinBox->value();

    openGLWidget->smRenderer->numberOfSpheres = value;
    ui->horizontalSlider->setValue(value);

    openGLWidget->update();
}

void MainWindow::onSlider2ValueChanged(int value)
{
    float floatValue = value * 0.05f;
    openGLWidget->smRenderer->sphereSize = floatValue;

    ui->spinBox_2->setValue(floatValue);

    openGLWidget->update();
}

void MainWindow::onSizeLabelEditingFinished()
{
    int value = ui->spinBox_2->value() * 20;

    openGLWidget->smRenderer->sphereSize = ui->spinBox_2->value();
    ui->horizontalSlider_2->setValue(value);

    openGLWidget->update();
}

void MainWindow::onSaveMesh() {
    QString qFileName = QFileDialog::getSaveFileName(this,
                                                     "Save Sphere Mesh",
                                                     QDir::homePath(),
                                                     "Sphere-mesh Files (*.sm);;All Files (*)");

    if (!qFileName.isEmpty()) {
        if (!qFileName.endsWith(".sm", Qt::CaseInsensitive))
            qFileName += ".sm";

        openGLWidget->sphereMesh->saveToFile(qFileName.toStdString().c_str());
    }
}

void MainWindow::onLoadMesh() {
    QString qFileName = QFileDialog::getOpenFileName(this, "Load Sphere Mesh", "", "Sphere-mesh Files (*.sm)");
    if (!qFileName.isEmpty()) {
        if (!qFileName.endsWith(".sm", Qt::CaseInsensitive))
            qFileName += ".sm";

        SphereMesh* sm = new SphereMesh();
        sm->loadFromFile(qFileName.toStdString().c_str());
        openGLWidget->setSphereMesh(sm);
        openGLWidget->update();
    }
}

void MainWindow::onDuplicateSphere()
{
    openGLWidget->duplicateSelectedSphere();
    openGLWidget->update();
}
