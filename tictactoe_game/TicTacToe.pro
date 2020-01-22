#-------------------------------------------------
#
# Project created by QtCreator 2016-10-26T14:28:04
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = TicTacToe
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    xotable.cpp \
    tablewindow.cpp

HEADERS  += mainwindow.h \
    xotable.h \
    tablewindow.h

FORMS    += mainwindow.ui \
    tablewindow.ui
