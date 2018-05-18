#include <QtTest>
#include <QCoreApplication>

#include "../Gui/gui.h"

// add necessary includes here

class gui_actions : public QObject
{
    Q_OBJECT

public:
    gui_actions();
    ~gui_actions();

private slots:
    void initTestCase();
    void cleanupTestCase();
    void test_case1();

};

gui_actions::gui_actions()
{

}

gui_actions::~gui_actions()
{

}

void gui_actions::initTestCase()
{

}

void gui_actions::cleanupTestCase()
{

}

void gui_actions::test_case1()
{

}

QTEST_MAIN(gui_actions)

#include "tst_gui_actions.moc"
