#include "tablewindow.h"
#include "ui_tablewindow.h"
#include "QTableWidget"
#include <QString>
#include <qmessagebox.h> ///For UpdateGameStatus()
#include <locale> ///For converting lowercase to uppercase in UpdateGameStatus()
#include "iostream"

TableWindow::TableWindow(QWidget *parent, XOtable *table_pointer) :
    QDialog(parent),
    ui(new Ui::TableWindow)
{
    ui->setupUi(this);
    table_p = table_pointer;
    parent_p = parent;
}

TableWindow::~TableWindow()
{
    delete ui;
}
void TableWindow::on_maintable_cellClicked(int row, int column)
{
    QTableWidgetItem *new_item;
    ///Is the cell going to 'x' or 'o' or 'n'(not available)
    char turn;
    turn = table_p->CellStatusAndMove(row, column);
    switch (turn)
    {
    case 'x':
        new_item = new QTableWidgetItem("X");
        break;
    case 'o':
        new_item = new QTableWidgetItem("O");
        break;
    case 'n':
        return;
    default:
        std::cerr<<"Internal Error: Invalid character for current status";
        abort();
    }

    new_item->setTextAlignment(Qt::AlignCenter);
    new_item->setForeground(QBrush(QColor(0,255,0,255),Qt::SolidPattern));
    ui->maintable->setItem(row,column,new_item);

    /// Update Table Status
    TableWindow::UpdateGameStatus();
}

void TableWindow::UpdateGameStatus()
{
    ///Check the current status(WinforXorO/Tie/none)
    char current_status = table_p->Status();
    ///Update the screen based on the status
    QMessageBox result_msg;
    switch (current_status)
    {
    case 'x':
        result_msg.setText("*** X has won! ***");
        result_msg.exec();
        this->close();
        break;
    case 'o':
        result_msg.setText("*** O has won! ***");
        result_msg.exec();
        this->close();
        break;
    case 't':
        result_msg.setText("It's a tie!");
        result_msg.exec();
        break;
    case 'n': {
        char turn = table_p->WhosTurn();
        std::locale loc;
        turn = std::toupper(turn,loc);
        QString status_bar_text;
        (status_bar_text += turn) += "'s turn";
        ui->label->setText((status_bar_text));
        }break;
    default:
        std::cerr<<"Internal Error: Invalid character for current status";
        abort();
    }
}



///Note: Items are not created automatically with the table, hence their
/// properties are not editable.
/*ui->maintable->item(row,column)->setText("X");
QTableWidgetItem *clicked_item;
clicked_item = ui->maintable->item(row,column);
if (clicked_item ==0)
    std::cout<<"Item creation failed"<<std::endl;*/
