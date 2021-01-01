//---------------------------------------------------------------------------
#ifndef serioptdlgH
#define serioptdlgH
//---------------------------------------------------------------------------
#include <QDialog>

#include "ui_serioptdlg.h"

class CmdOptDialog;
//---------------------------------------------------------------------------
class SerialOptDialog : public QDialog, private Ui::SerialOptDialog
{
    Q_OBJECT

protected:
    void  showEvent(QShowEvent *);

    CmdOptDialog *cmdOptDialog;

    void  UpdatePortList(void);
    void  UpdateEnable(void);

public slots:
    void  BtnOkClick();
    void  BtnCmdClick();
    void  OutTcpPortClick();

public:
    QString Path,Cmds[3];
	int Opt,CmdEna[3];

    explicit SerialOptDialog(QWidget*);
};
//---------------------------------------------------------------------------
#endif
