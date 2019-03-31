/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */

// HeightMapGenDlg.h : header file
//

#pragma once
#include "afxcmn.h"
#include "afxwin.h"
#include "PictureCtrl.h"

#define IMGVIEW_W 320
#define IMGVIEW_H 320


// CHeightMapGenDlg dialog
class CHeightMapGenDlg : public CDialog
{
// Construction
public:
	CHeightMapGenDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_HEIGHTMAPGEN_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedGenerateHeightmap();
	DWORD m_width;
	DWORD m_height;
	CString m_interpolator;
	CString m_saveas;
	afx_msg void OnBnClickedSaveasBrowse();
	CString m_persistence;
	BOOL m_smoothing;
	CProgressCtrl m_progress;

public:
	afx_msg void OnBnClickedPinRect();
	afx_msg void OnEnChangeWidth();
	static void cbProgress(int progress);
	void SetDefault(void);
	CString m_octave;
	CPictureCtrl m_imgView;
};
