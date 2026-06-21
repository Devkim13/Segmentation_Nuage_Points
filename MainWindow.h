#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>

class QPushButton;
class QLabel;
class QTextEdit;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void chooseFile();
    void runProcessing();
    void openResultsFolder();
    void compareRawAndPreprocessed();

private:
    QString selectedFile;

    QLabel* fileLabel = nullptr;
    QTextEdit* logTextEdit = nullptr;

    QPushButton* loadButton = nullptr;
    QPushButton* runButton = nullptr;
    QPushButton* compareButton = nullptr;
    QPushButton* openFolderButton = nullptr;

    bool hasResults = false;
};

#endif