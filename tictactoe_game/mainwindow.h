#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <tablewindow.h>
#include <xotable.h>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void CreateNewTableWindow();

    void Resume();

    void on_StartBtn_clicked();

    void on_ResumeBtn_clicked();

    void on_ExitBtn_clicked();

private:
    Ui::MainWindow *ui;
    XOtable *table_pointer;
    TableWindow *w2;
};

#endif // MAINWINDOW_H
