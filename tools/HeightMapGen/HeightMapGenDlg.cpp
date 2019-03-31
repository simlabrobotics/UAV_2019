/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */

// HeightMapGenDlg.cpp : implementation file
//

#include "stdafx.h"
#include "HeightMapGen.h"
#include "HeightMapGenDlg.h"
#include "Perlin.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


static CHeightMapGenDlg* __dlg__ = NULL;


// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()


// CHeightMapGenDlg dialog




CHeightMapGenDlg::CHeightMapGenDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CHeightMapGenDlg::IDD, pParent)
	, m_width(512)
	, m_height(512)
	, m_interpolator(_T(""))
	, m_saveas(_T(""))
	, m_persistence(_T(""))
	, m_smoothing(FALSE)
	, m_octave(_T(""))
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	__dlg__ = this;
}

void CHeightMapGenDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_WIDTH, m_width);
	DDX_Text(pDX, IDC_HEIGHT, m_height);
	DDX_CBString(pDX, IDC_INTERPOLATOR, m_interpolator);
	DDX_Text(pDX, IDC_SAVEAS, m_saveas);
	DDX_CBString(pDX, IDC_PERSISTENCE, m_persistence);
	DDX_Check(pDX, IDC_SMOOTHING, m_smoothing);
	DDX_Control(pDX, IDC_PROGRESS, m_progress);
	DDX_CBString(pDX, IDC_OCTAVE, m_octave);
	DDX_Control(pDX, IDC_STATIC_IMAGE, m_imgView);
}

BEGIN_MESSAGE_MAP(CHeightMapGenDlg, CDialog)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDOK, &CHeightMapGenDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_GENERATE_HEIGHTMAP, &CHeightMapGenDlg::OnBnClickedGenerateHeightmap)
	ON_BN_CLICKED(IDC_SAVEAS_BROWSE, &CHeightMapGenDlg::OnBnClickedSaveasBrowse)
	ON_BN_CLICKED(IDC_PIN_RECT, &CHeightMapGenDlg::OnBnClickedPinRect)
	ON_EN_CHANGE(IDC_WIDTH, &CHeightMapGenDlg::OnEnChangeWidth)
END_MESSAGE_MAP()


// CHeightMapGenDlg message handlers

BOOL CHeightMapGenDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	SetDefault();

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CHeightMapGenDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CHeightMapGenDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CHeightMapGenDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CHeightMapGenDlg::OnBnClickedOk()
{
	// TODO: Add your control notification handler code here
	//OnOK();
}

void CHeightMapGenDlg::OnBnClickedGenerateHeightmap()
{
	UpdateData(TRUE);

	if (m_width <= 0)
	{
		::AfxMessageBox(_T("Set width with positive number."), MB_OK);
		GetDlgItem(IDC_WIDTH)->SetFocus();
		return;
	}
	else if (m_height <= 0)
	{
		::AfxMessageBox(_T("Set height with positive number."), MB_OK);
		GetDlgItem(IDC_HEIGHT)->SetFocus();
		return;
	}

	if (m_persistence.IsEmpty())
	{
		::AfxMessageBox(_T("Set persistence."), MB_OK);
		GetDlgItem(IDC_PERSISTENCE)->SetFocus();
		return;
	}

	if (m_interpolator.IsEmpty())
	{
		::AfxMessageBox(_T("Set interpolation method."), MB_OK);
		GetDlgItem(IDC_INTERPOLATOR)->SetFocus();
		return;
	}

	if (m_octave.IsEmpty())
	{
		::AfxMessageBox(_T("Set number of iterations(octaves)."), MB_OK);
		GetDlgItem(IDC_OCTAVE)->SetFocus();
		return;
	}

	if (m_saveas.IsEmpty())
	{
		::AfxMessageBox(_T("Set output filename."), MB_OK);
		GetDlgItem(IDC_SAVEAS)->SetFocus();
		return;
	}

	m_progress.SetPos(0);
	
	float persistence = 1.0f;
	ePerlinInterpolator interpolator = ePI_COSINE;
	int octave = 1;

	if (m_persistence == _T("1/8"))
		persistence = 1.0f/8;
	else if (m_persistence == _T("1/4"))
		persistence = 1.0f/4;
	else if (m_persistence == _T("1/2"))
		persistence = 1.0f/2;
	else if (m_persistence == _T("2/3"))
		persistence = 2.0f/3;
	else if (m_persistence == _T("3/4"))
		persistence = 3.0f/4;
	else if (m_persistence == _T("7/8"))
		persistence = 7.0f/8;
	else if (m_persistence == _T("9/10"))
		persistence = 9.0f/10;
	else if (m_persistence == _T("1"))
		persistence = 1.0f;

	if (m_interpolator == _T("Linear"))
		interpolator = ePI_LINEAR;
	else if (m_interpolator == _T("Cosine"))
		interpolator = ePI_COSINE;
	else if (m_interpolator == _T("Cubic"))
		interpolator = ePI_CUBIC;

	if (m_octave == _T("1"))
		octave = 1;
	else if (m_octave == _T("2"))
		octave = 2;
	else if (m_octave == _T("4"))
		octave = 4;
	else if (m_octave == _T("6"))
		octave = 6;
	else if (m_octave == _T("8"))
		octave = 8;
	else if (m_octave == _T("10"))
		octave = 10;
	

#ifdef UNICODE
	char saveas[MAX_PATH];
	size_t converted;
	wcstombs_s(&converted, saveas, MAX_PATH, m_saveas, _TRUNCATE);
	int rtn = GenPerlin(m_width, m_height, persistence, interpolator, octave, (m_smoothing ? true : false), saveas, cbProgress);
#else
	int rtn = GenPerlin(m_width, m_height, persistence, interpolator, octave, (m_smoothing ? true : false), (LPCTSTR)m_saveas);
#endif

	if (rtn != eP_SUCCESS)
		::AfxMessageBox(_T("Failed."), MB_OK);
	else
		m_imgView.Load(m_saveas);

	//m_progress.SetPos(0);
}

void CHeightMapGenDlg::OnBnClickedSaveasBrowse()
{
	UpdateData(TRUE);
	CFileDialog dlg(FALSE, NULL, NULL, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT);
	if(dlg.DoModal() == IDOK)
    {
		m_saveas = dlg.GetPathName();
		UpdateData(FALSE);
	}
}

void CHeightMapGenDlg::OnBnClickedPinRect()
{
	if (BST_CHECKED == ((CButton*)GetDlgItem(IDC_PIN_RECT))->GetCheck())
	{
		UpdateData(TRUE);
		m_height = m_width;
		UpdateData(FALSE);
		GetDlgItem(IDC_HEIGHT)->EnableWindow(FALSE);
	}
	else
	{
		GetDlgItem(IDC_HEIGHT)->EnableWindow(TRUE);
	}
}

void CHeightMapGenDlg::OnEnChangeWidth()
{
	if (BST_CHECKED == ((CButton*)GetDlgItem(IDC_PIN_RECT))->GetCheck())
	{
		UpdateData(TRUE);
		m_height = m_width;
		UpdateData(FALSE);
	}
}

void CHeightMapGenDlg::cbProgress(int progress)
{
	if (__dlg__)
		__dlg__->m_progress.SetPos(progress);
}

void CHeightMapGenDlg::SetDefault(void)
{
	((CButton*)GetDlgItem(IDC_PIN_RECT))->SetCheck(BST_CHECKED);
	GetDlgItem(IDC_HEIGHT)->EnableWindow(FALSE);
	((CComboBox*)GetDlgItem(IDC_PERSISTENCE))->SelectString(-1, _T("1/8"));
	((CComboBox*)GetDlgItem(IDC_INTERPOLATOR))->SelectString(-1, _T("Cosine"));
	((CComboBox*)GetDlgItem(IDC_OCTAVE))->SelectString(-1, _T("4"));
	UpdateData(TRUE);

	m_width = 512;
	m_height = 512;
	UpdateData(FALSE);
}
