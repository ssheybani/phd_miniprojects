#ifndef XOTABLE_H
#define XOTABLE_H
#define TABLE_LENGTH 3
#include "vector"

class XOcell
{
public:
    XOcell(int new_row, int new_column, char new_owner){row = new_row;column = new_column; owner = new_owner;};
    XOcell(){};
    int row;
    int column;
    char owner;//Possible values: x,o,n

    bool operator==(const XOcell& c2){if ((row==c2.row)&&(column==c2.column)) return true;
                                     return false;}
};

class XOtable
{
    ///Singleton class for the X-O Table
public:
    static XOtable *GetInstance();
    static void DestroyInstance();

    char CellStatusAndMove(int row, int column);
    char Status(); /// Use last_movement variable.
    char WhosTurn(); ///Use last_movement variable.
    bool isOccupied(int row, int column);
    //int getWinner(){return winner;};

private:
    XOtable();
    ~XOtable();

    //bool addCell(XOcell new_cell);
    std::vector<XOcell> cells;
    XOcell last_movement;
    static XOtable *the_table;
};

#endif // XOTABLE_H
