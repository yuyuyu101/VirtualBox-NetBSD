/* Copyright (c) 2001, Stanford University
 * All rights reserved
 *
 * See the file LICENSE.txt for information on redistributing this software.
 */

#include "cr_error.h"
#include "cr_process.h"
#include "cr_string.h"
#include "cr_mem.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <signal.h>
#ifndef WINDOWS
#include <unistd.h>
#else
#pragma warning ( disable : 4127 )
#define snprintf _snprintf
#endif

/**
 * Sleep/pause for the given number of seconds.
 */
void crSleep( unsigned int seconds )
{
#ifdef WINDOWS
  Sleep(seconds*1000); /* milliseconds */
#else
  sleep(seconds);
#endif
}

/**
 * Sleep/pause for the given number of milliseconds.
 */
void crMsleep( unsigned int msec )
{
#ifdef WINDOWS
     Sleep(msec); 
#else
     usleep(msec*1000); /* usecs */
#endif
}


/*
 * Spawn (i.e. fork/exec) a new process.
 */
CRpid crSpawn( const char *command, const char *argv[] )
{
#ifdef WINDOWS
	char newargv[1000];
	int i;
	STARTUPINFO si;
	PROCESS_INFORMATION pi;

	(void) command;

	ZeroMemory( &si, sizeof(si) );
	si.cb = sizeof(si);
	ZeroMemory( &pi, sizeof(pi) );

	crStrncpy(newargv, argv[0], 1000 );
	for (i = 1; argv[i]; i++) {
		crStrcat(newargv, " ");
		crStrcat(newargv, argv[i]);
	}

	if ( !CreateProcess(NULL, newargv, NULL, NULL, FALSE, 0, NULL,
				NULL, &si, &pi) )
	{
		crWarning("crSpawn failed, %d", GetLastError());
		return 0;
	}
	return pi.hProcess;
#else
	pid_t pid;
	if ((pid = fork()) == 0)
	{
		/* I'm the child */
		int err = execvp(command, (char * const *) argv);
		crWarning("crSpawn failed (return code: %d)", err);
		return 0;
	}
	return (unsigned long) pid;
#endif
}


/*
 * Kill the named process.
 */
void crKill( CRpid pid )
{
#ifdef WINDOWS
	TerminateProcess( pid, 0 );
#else
	kill((pid_t) pid, SIGKILL);
#endif
}


/*
 * Return the name of the running process.
 * name[0] will be zero if anything goes wrong.
 */
void crGetProcName( char *name, int maxLen )
{
#ifdef WINDOWS
	char command[1000];
	int c = 0;

	*name = 0;
	
	if (!GetModuleFileName( NULL, command, maxLen ))
		return;

	while (1) {
		/* crude mechanism to blank out the backslashes
		 * in the Windows filename and recover the actual
		 * program name to return */
		if (crStrstr(command, "\\")) {
			crStrncpy(name, command+c+1, maxLen);
			command[c] = 32;
			c++;
		}
		else
			break;
	}
#else
	/* Unix:
	 * Call getpid() to get our process ID.
	 * Then use system() to write the output of 'ps' to a temp file.
	 * Read/scan the temp file to map the process ID to process name.
	 * I'd love to find a better solution! (BrianP)
	 */
	FILE *f;
	pid_t pid = getpid();
	char *tmp, command[1000];

	/* init to NULL in case of early return */
	*name = 0;

	/* get a temporary file name */
	tmp = tmpnam(NULL);
	if (!tmp)
		return;
	/* pipe output of ps to temp file */
#ifndef SunOS
# ifdef VBOX
	snprintf(command, sizeof(command), "ps > %s", tmp);
# else
	sprintf(command, "ps > %s", tmp);
# endif
#else
# ifdef VBOX
	snprintf(command, sizeof(command), "ps -e -o 'pid tty time comm'> %s", tmp);
# else
	sprintf(command, "ps -e -o 'pid tty time comm'> %s", tmp);
# endif
#endif
	system(command);

	/* open/scan temp file */
	f = fopen(tmp, "r");
	if (f) {
		char buffer[1000], cmd[1000], *psz, *pname;
		while (!feof(f)) {
			int id;
			fgets(buffer, 999, f);
			sscanf(buffer, "%d %*s %*s %999s", &id, cmd);
			if (id == pid) {
				for (pname=psz=&cmd[0]; *psz!=0; psz++)
				{
					switch (*psz)
					{
						case '/':
						pname = psz+1;
						break;
					}
				}
				crStrncpy(name, pname, maxLen);
				break;
			}
		}
		fclose(f);
	}
	remove(tmp);
#endif
}


/*
 * Return current directory string.
 */
void crGetCurrentDir( char *dir, int maxLen )
{
#ifdef WINDOWS
  if (!GetCurrentDirectory(maxLen, dir))
	dir[0] = 0;
#else
  if (!getcwd(dir, maxLen))
	dir[0] = 0;
#endif
}


/**
 * Return current process ID number.
 */
CRpid crGetPID(void)
{
#ifdef WINDOWS 
  //return _getpid();
  return GetCurrentProcess();
#else
  return getpid();
#endif
}


#if 0
/* simple test harness */
int main(int argc, char **argv)
{
   char name[100];
   printf("argv[0] = %s\n", argv[0]);

   crGetProcName(name, 100);
   printf("crGetProcName returned %s\n", name);

   return 0;
}
#endif
