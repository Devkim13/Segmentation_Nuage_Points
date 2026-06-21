#include "MainWindow.h"
#include "PointCloudPipeline.h"

#include <QDesktopServices>
#include <QDir>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTextEdit>
#include <QUrl>
#include <QVBoxLayout>
#include <QWidget>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("Segmentation et graphe hiérarchique de nuage de points");
    resize(1000, 700);

    QWidget* centralWidget = new QWidget(this);

    QVBoxLayout* mainLayout = new QVBoxLayout(centralWidget);

    fileLabel = new QLabel("Aucun fichier sélectionné");
    fileLabel->setWordWrap(true);

    loadButton = new QPushButton("Charger un fichier .PLY / .PCD");
    runButton = new QPushButton("Lancer le traitement");
    openFolderButton = new QPushButton("Ouvrir le dossier des résultats");

    compareButton = new QPushButton("Comparer brut / prétraité");
    compareButton->setEnabled(false);

    runButton->setEnabled(false);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(loadButton);
    buttonLayout->addWidget(runButton);
    buttonLayout->addWidget(compareButton);
    buttonLayout->addWidget(openFolderButton);

    logTextEdit = new QTextEdit();
    logTextEdit->setReadOnly(true);
    logTextEdit->setPlaceholderText("Les logs du traitement seront affichés ici...");

    mainLayout->addWidget(fileLabel);
    mainLayout->addLayout(buttonLayout);
    mainLayout->addWidget(logTextEdit);

    setCentralWidget(centralWidget);

    connect(loadButton, &QPushButton::clicked, this, &MainWindow::chooseFile);
    connect(runButton, &QPushButton::clicked, this, &MainWindow::runProcessing);
    connect(openFolderButton, &QPushButton::clicked, this, &MainWindow::openResultsFolder);
    connect(compareButton, &QPushButton::clicked, this, &MainWindow::compareRawAndPreprocessed);
}

void MainWindow::chooseFile()
{
    QString fileName = QFileDialog::getOpenFileName(
        this,
        "Choisir un nuage de points",
        QDir::homePath(),
        "Nuages de points (*.ply *.pcd)"
    );

    if (fileName.isEmpty())
    {
        return;
    }

    selectedFile = fileName;

    fileLabel->setText("Fichier sélectionné : " + selectedFile);
    logTextEdit->append("Fichier chargé : " + selectedFile);
    runButton->setEnabled(true);
}

void MainWindow::runProcessing()
{
    if (selectedFile.isEmpty())
    {
        QMessageBox::warning(
            this,
            "Aucun fichier",
            "Veuillez d'abord charger un fichier .ply ou .pcd."
        );
        return;
    }

    logTextEdit->clear();
    logTextEdit->append("Début du traitement...");
    logTextEdit->append("Fichier : " + selectedFile);
    logTextEdit->append("----------------------------------------");

    loadButton->setEnabled(false);
    runButton->setEnabled(false);

    PointCloudPipeline pipeline;
    PipelineResult result = pipeline.run(selectedFile.toStdString());

    logTextEdit->append(QString::fromStdString(result.logs));

    if (result.success)
    {
        hasResults = true;
        compareButton->setEnabled(true);

        logTextEdit->append("----------------------------------------");
        logTextEdit->append("Traitement terminé avec succès.");
        logTextEdit->append("Fichiers générés :");
        logTextEdit->append("- primitives.json");
        logTextEdit->append("- hierarchy.json");
        logTextEdit->append("- primitives_colored.ply");

        QMessageBox::information(
            this,
            "Traitement terminé",
            "Les résultats ont été générés :\n"
            "- primitives.json\n"
            "- hierarchy.json\n"
            "- primitives_colored.ply"
        );
    }
    else
    {
        logTextEdit->append("----------------------------------------");
        logTextEdit->append("Erreur pendant le traitement.");

        QMessageBox::critical(
            this,
            "Erreur",
            "Le traitement a échoué. Vérifiez les logs."
        );
    }

    loadButton->setEnabled(true);
    runButton->setEnabled(true);
}

void MainWindow::openResultsFolder()
{
    QString path = QDir::currentPath();

    QDesktopServices::openUrl(
        QUrl::fromLocalFile(path)
    );
}
void MainWindow::compareRawAndPreprocessed()
{
    if (!hasResults)
    {
        QMessageBox::warning(
            this,
            "Aucun résultat",
            "Veuillez d'abord lancer le traitement."
        );
        return;
    }

    logTextEdit->append("Ouverture de la fenêtre de comparaison brut / prétraité...");

    PointCloudPipeline::visualizeRawAndPreprocessed(
        "raw_cloud.ply",
        "preprocessed_cloud.ply"
    );
}