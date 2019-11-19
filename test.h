/*
* 
* Name				Lawrence Bush
* Student Number	660 220 742
* Email				bushl2@rpi.edu, Lawrence_Bush@dps.state.ny.us
* Class				Distributed Algorithms and Systems (Spring 2002)
* Professor			Costas Busch
*
* Final Project	Implement a Lock-Free Linked List (based on Valois' paper)
*
*/
//
//////////////////////////////////////////////////////////////////////

//	Description of Program Files
//
//		File		Purpose
//		----		-------
//
//		test.h:		Header file for the test class.
//					It declares the interface for the test class.
//		
//
//
//		
//******************************************************************************************
//******************************************************************************************
//******************************************************************************************
//*************                                                               **************
//*************                           Test Object                         **************
//*************                                                               **************
//*************                           Header File                         **************
//*************                                                               **************
//******************************************************************************************
//******************************************************************************************
//******************************************************************************************
//
//////////////////////////////////////////////////////////////////////

// If def to prevent multiple compilations.
#if !defined(AFX_test_H__A892AC06_6934_41DF_A784_5C8E45F6EEED__INCLUDED_)
#define AFX_test_H__A892AC06_6934_41DF_A784_5C8E45F6EEED__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

// include files
#include <stdio.h>
#include <process.h>
#include "CriticalSection.h"
#include "Lock.h"
#include "LockFreeList.h"

// thread stuff
typedef unsigned (WINAPI  *PBEGINTHREADEX_THREADFUNC) (
													   LPVOID LPTHREADPARAMETER
													   );
typedef unsigned *PBEGINTHREADEX_THREADID;
// end thread stuff

// *************************************
// *************************************
// ***                               ***
// ***          Test Class           *** 
// ***          Declaration          *** 
// ***                               ***
// *************************************
// *************************************
class test {
	
private:
	// Initializes the class interface functions.
	int mv_my_number; // Gives each thread a unique number.
	int mv_number_of_threads;// Stores the number of threads.
	int net_added_nodes; // Sum of all threads.
	LockFreeList<int> testLockFreeList1; // The list object.
	//bool stop;// = 0; // should thread stop? 1=yes

private:
	// This points to a critical section (my construct)
	// It synchronizes my_number
	CriticalSection m_sync_my_number;
	// This points to a critical section (my construct)
	// It synchronizes output to the screen
	CriticalSection m_sync_output;

public:
	
	// This is a funny declaration for a thread within a class.
	static DWORD WINAPI ThreadFunc (LPVOID param);

	// Initializes constructors, destructors 
	// and the class interface functions.
	test();
	virtual ~test();

	// Various testing function declarations.
	void multithreaded_test(int number_of_threads);
	int  testHelper(int my_number);
	int	 integrityTest(int num_threads);
	int  test::TestFuctionF(int my_number); // This is the one we use.
	int  test::print_test_summary();

	
	int  get_my_number(); // Sycronizes the giving of a unique thread number.
};

#endif // !defined(AFX_test_H__A892AC06_6934_41DF_A784_5C8E45F6EEED__INCLUDED_)
