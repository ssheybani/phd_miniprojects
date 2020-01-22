#ifndef TABLEWINDOW_H
#define TABLEWINDOW_H

#include <QDialog>
#include <xotable.h>
#include <QAbstractButton>
#include <QLabel>

namespace Ui {
class TableWindow;
}

class TableWindow : public QDialog
{
    Q_OBJECT

public:
    explicit TableWindow(QWidget *parent = 0, XOtable *table_pointer = 0);
    ~TableWindow();

private slots:
    void on_maintable_cellClicked(int row, int column);

    void UpdateGameStatus();

private:
    Ui::TableWindow *ui;
    XOtable *table_p;
    QWidget *parent_p;
};

#endif // TABLEWINDOW_H
