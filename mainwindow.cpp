#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "openglwidget.h"

#include <QAction>
#include <QIcon>
#include <QDir>
#include <QCheckBox>
#include <QFileDialog>
#include <QWidgetAction>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow), openGLWidget(new OpenGLWidget(this))
{
    ui->setupUi(this);
    setCentralWidget(openGLWidget);

    QString iconPath = QDir::currentPath() + "/icons/";

    ui->topToolBar->setIconSize(QSize(24, 24));
    ui->leftToolBar->setIconSize(QSize(24, 24));

    QAction *addButton = new QAction(QIcon(iconPath + "plus.png"), "Add", this);
    QAction *deleteButton = new QAction(QIcon(iconPath + "eraser.png"), "Delete", this);
    QAction *linkCap = new QAction(QIcon(iconPath + "linkCap.png"), "Link Capsule", this);
    QAction *linkTri = new QAction(QIcon(iconPath + "linkTri.png"), "Link Prysmoid", this);
    QAction *unlinkCap = new QAction(QIcon(iconPath + "deleteLinkCap.png"), "Delete Capsule Link", this);
    QAction *unlinkTri = new QAction(QIcon(iconPath + "deleteLinkTri.png"), "Delete Prysmoid Link", this);

    connect(addButton, &QAction::triggered, this, &MainWindow::onDuplicateSphere);
    connect(deleteButton, &QAction::triggered, this, &MainWindow::onDeleteSphere);
    connect(linkCap, &QAction::triggered, this, &MainWindow::onLinkCap);
    connect(linkTri, &QAction::triggered, this, &MainWindow::onLinkTri);
    connect(unlinkCap, &QAction::triggered, this, &MainWindow::onUnlinkCap);
    connect(unlinkTri, &QAction::triggered, this, &MainWindow::onUnlinkTri);

    ui->leftToolBar->addAction(addButton);
    ui->leftToolBar->addAction(deleteButton);
    ui->leftToolBar->addAction(linkCap);
    ui->leftToolBar->addAction(linkTri);
    ui->leftToolBar->addAction(unlinkCap);
    ui->leftToolBar->addAction(unlinkTri);

    QAction *topAction1 = new QAction(QIcon(iconPath + "save.png"), "Save...", this);
    QAction *topAction2 = new QAction(QIcon(iconPath + "import.png"), "Import...", this);
    QAction *topAction3 = new QAction(QIcon(iconPath + "export.png"), "Export Geometry...", this);
    QAction *topAction4 = new QAction(QIcon(iconPath + "undo.png"), "Undo", this);

    QWidgetAction *separatorAction = new QWidgetAction(this);
    QFrame *separator = new QFrame();
    separator->setFrameShape(QFrame::VLine);
    separator->setFrameShadow(QFrame::Sunken);
    separatorAction->setDefaultWidget(separator);

    connect(topAction1, &QAction::triggered, this, &MainWindow::onSaveMesh);
    connect(topAction2, &QAction::triggered, this, &MainWindow::onLoadMesh);
    connect(topAction3, &QAction::triggered, this, &MainWindow::onExportGeometry);
    connect(topAction4, &QAction::triggered, this, &MainWindow::onUndo);

    ui->topToolBar->addAction(topAction1);
    ui->topToolBar->addAction(topAction2);
    ui->topToolBar->addAction(topAction3);
    ui->topToolBar->addAction(separatorAction);
    ui->topToolBar->addAction(topAction4);

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
    connect(ui->checkBox_3, &QCheckBox::stateChanged, this, &MainWindow::onCheckMeshVisible);

    connect(ui->horizontalSlider, &QSlider::valueChanged, this, &MainWindow::onSlider1ValueChanged);
    connect(ui->spinBox, QOverload<int>::of(&QSpinBox::valueChanged), ui->horizontalSlider, &QSlider::setValue);

    connect(ui->horizontalSlider_2, &QSlider::valueChanged, this, &MainWindow::onSlider2ValueChanged);
    connect(ui->spinBox_2, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onSizeLabelEditingFinished);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onUndo()
{
    openGLWidget->undo();
    openGLWidget->update();
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

void MainWindow::onCheckMeshVisible(int state)
{
    if (state == Qt::Checked)
        openGLWidget->mesh->isVisible = true;
    else
        openGLWidget->mesh->isVisible = false;

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

    if (floatValue >= 0.5f)
    {
        openGLWidget->smRenderer->connectivitySize = std::clamp(2 * (floatValue - 0.5f), 0.05f, 1.0f);
        openGLWidget->smRenderer->sphereSize = 1.0f;
    }
    else
        openGLWidget->smRenderer->sphereSize = (floatValue * 2);

    ui->spinBox_2->setValue(floatValue);

    openGLWidget->update();
}

void MainWindow::onSizeLabelEditingFinished()
{
    int value = ui->spinBox_2->value() * 20;

    // openGLWidget->smRenderer->connectivitySize = ui->spinBox_2->value();
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

void MainWindow::onDeleteSphere()
{
    openGLWidget->deleteSelectedSphere();
    openGLWidget->update();
}

void MainWindow::onLinkCap()
{
    openGLWidget->linkCap();
}

void MainWindow::onLinkTri()
{
    openGLWidget->linkTri();
}

void MainWindow::onUnlinkCap()
{
    openGLWidget->unlinkCap();
}

void MainWindow::onUnlinkTri()
{
    openGLWidget->unlinkTri();
}

void MainWindow::onExportGeometry() {
    QString objData = QString::fromStdString(openGLWidget->smRenderer->exportGeometryAsString());

    QString filePath = QFileDialog::getSaveFileName(this, "Save Geometry", "", "OBJ Files (*.obj)");

    if (!filePath.isEmpty()) {
        QFile file(filePath);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream out(&file);
            out << objData;
            file.close();
            QMessageBox::information(this, "Success", "Geometry exported successfully.");
        } else {
            QMessageBox::critical(this, "Error", "Failed to save the file.");
        }
    }
}

