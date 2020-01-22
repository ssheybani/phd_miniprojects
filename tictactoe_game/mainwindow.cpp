#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <tablewindow.h>
#include "iostream"
#include <xotable.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    w2 = NULL; /// Apparently this is necessary!
}

MainWindow::~MainWindow()
{
    delete ui;
    delete w2;
    XOtable::DestroyInstance();
}

void MainWindow::CreateNewTableWindow()
{
    XOtable::DestroyInstance();/// X-O Table is singleton
    std::clog<<"CreateNewTableWindow: XOtable deleted"<<std::endl;
    table_pointer = NULL;
    if (w2 != NULL){
        delete w2;
        std::clog<<"CreateNewTableWindow: w2 deleted"<<std::endl;
        w2 = NULL;}
    std::clog<<"CreateNewTableWindow: Pointers deleted"<<std::endl;

    table_pointer = XOtable::GetInstance();
    w2 = new TableWindow(this,table_pointer);
}

void MainWindow::Resume()
{
    w2->show();
}

void MainWindow::on_StartBtn_clicked()
{
    MainWindow::CreateNewTableWindow();
    std::clog<<"on_StartBtn_clicked: New table window created"<<std::endl;
    MainWindow::Resume();
}

void MainWindow::on_ResumeBtn_clicked()
{
    MainWindow::Resume();
}

void MainWindow::on_ExitBtn_clicked()
{
    this->close();
}


