#include "xotable.h"
#include "iostream"
#include "cstdlib"
#include "algorithm"

XOtable::XOtable()
{
    ///Implement it
    last_movement = XOcell(0,0,'o'); /// In order to make sure WhosTurn() works
                                     /// properly in the beginning.
}

XOtable::~XOtable()
{
    ///Implement it
}


XOtable *XOtable::the_table = NULL;

XOtable *XOtable::GetInstance()
{
    if (the_table == NULL)
    {
        the_table = new XOtable();
        std::clog<<"One instance of table created"<<std::endl;
    }
    return the_table;
}

void XOtable::DestroyInstance()
{
      delete the_table;
      the_table = NULL; // so GetInstance will still work.
}

char XOtable::CellStatusAndMove(int row, int column)
{
    /// Evaluates the proposed move and returns a character relative to the result
    /// of such a move. The candidates for the character are:
    ///     'x': It has been X's turn and this movement is submitted as its movement.
    ///     'o': It has been O's turn and this movement is submitted as its movement.
    ///     'n': The corresponding cell has already been occupied.

    if (the_table->isOccupied(row, column))
        return 'n';
    last_movement = XOcell(row, column, the_table->WhosTurn());
    the_table->cells.push_back(last_movement);
    return last_movement.owner;
}

char XOtable::Status()
{
    /// Warning: In order to receive reliable result, this method must be
    ///     called after each move.
    /// This method evaluates current status of the table, only resulted from the last
    /// move(For efficiency) and returns a character:
    ///     x: X has won.
    ///     o: O has won.
    ///     t: It's a tie.
    ///     n: None. the game has not finished yet.

    std::vector<XOcell>::iterator it;
    int cnt_row = 0;
    int cnt_column = 0;
    int cnt_primarydiagonal = 0;
    int cnt_secondarydiagonal = 0;
    for (it = this->cells.begin(); it != this->cells.end(); it++)
    {
        if (it->owner==last_movement.owner)
        {
            if (it->row==last_movement.row)
                cnt_row++;
            if (it->column==last_movement.column)
                cnt_column++;
            if ( (last_movement.row==last_movement.column) && (it->row == it->column) )
                cnt_primarydiagonal++;
            if ( (last_movement.row== TABLE_LENGTH-1-last_movement.column)
                 && (it->row == TABLE_LENGTH-1- it->column) )
                cnt_secondarydiagonal++;
        }
    }
    if (std::max(cnt_row, std::max(cnt_column, std::max(cnt_primarydiagonal,cnt_secondarydiagonal))) == 3)
        return last_movement.owner;
    if (cells.size()==9)
        return 't';
    return 'n';
}
char XOtable::WhosTurn()
{
    /// Warning: In order to work properly in the beginning of the game,
    /// last_movement.owner must be initialized with 'o'.

    switch(last_movement.owner)
    {
    case 'x':
        return 'o';
        //break;
    case 'o':
        return 'x';
        //break;
    default:
        std::cerr<<"Internal Error: table->last_movement->owner contains unknown character"<<std::endl;
        abort();
    }
}

bool XOtable::isOccupied(int row, int column)
{
    XOcell proposed_cell(row,column,'x');/// The owner of this cell is not important,
                                         /// because == is overloaded to only look at
                                         /// row and column for comparison between cells.
    if ( std::find(this->cells.begin(),this->cells.end(),proposed_cell)
        != this->cells.end() )
        return true;
    return false;
}

