/* $Id$ */
/*
******************************************************************************
**  CarMaker
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
*/

#include <Global.h>

#if defined(WIN32) && !defined(INTIME)
#  include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <infoc.h>
#include <CarMaker.h>
#include <ipgdriver.h>
#include <road.h>

extern const char *SetConnectedIO (const char *io);

static const char *CompileLibs[] = {
    /* /local/work.pt/cm/src_lib/Portings/win32/lib/libcar.a */
    /* /local/work.pt/cm/src_lib/Portings/win32/lib/libcarmaker.a */
    /* /local/work.pt/cm/lib/driver/win32/lib/libipgdriver.a */
    /* /local/work.pt/cm/lib/road/win32/lib/libipgroad.a */
    /* /local/work.pt/cm/lib/tire/win32/lib/libipgtire.a */
    /* /local/work.pt/cm/lib/tametire/win32/libtametire.a */
    "libcar.a	CarMaker-Car win32 6.0.3 2017-09-27",
    "libcarmaker.a	CarMaker win32 6.0.3 2017-09-27",
    "libipgdriver.a	IPGDriver win32 6.7.0 2017-08-16",
    "libipgroad.a	IPGRoad win32 6.0.3 2017-09-22",
    "libipgtire.a	IPGTire win32 5.0.2 2017-05-17",
    "libtametire.a	TameTire win32 3.1.1 2017-04-25",
    NULL
};


static const char *CompileFlags[] = {
    "-O3 -march=i686 -DNDEBUG -DWIN32 -DCM_NUMVER=60003",
    "-DMYMODELS -DWITH_ADTF -Wall -Wimplicit",
    "-Wmissing-prototypes -D__USE_MINGW_ANSI_STDIO",
    NULL
};


tAppStartInfo   AppStartInfo = {
    "CarMaker 6.0.3 - Car_Generic",          /* App_Version         */
    "33",          /* App_BuildVersion    */
    "pt",     /* App_CompileUser     */
    "pluto.ipg",         /* App_CompileSystem   */
    "2017-09-27 12:31:10",  /* App_CompileTime */

    CompileFlags,   /* App_CompileFlags  */
    CompileLibs,    /* App_Libs          */

    /* NULL, */     /* User              */

    NULL,           /* TestRunName       */
    NULL,           /* TestRunFName      */
    NULL,           /* TestRunVariation  */
    NULL,           /* LogFName          */

    0,              /* SaveMode          */
    0,              /* TimeStat          */
    0,              /* OnErrSaveHist     */

    0,              /* Verbose           */
    0,              /* Comments          */
    0,              /* ModelCheck        */
    0,              /* Snapshot          */
    0,              /* DriverAdaption    */
    0,              /* Log2Screen        */
    0,              /* ShowDataDict      */
    0,              /* DontHandleSignals */
    {0, 0, NULL},   /* Suffixes          */
    {0, 0, NULL}    /* Keys              */
};



void
App_InfoPrint (void)
{
    int i;
    Log ("App.Version\t%s #%s (%s)\n",
            AppStartInfo.App_Version,
            AppStartInfo.App_BuildVersion,
            SimCoreInfo.Version);
    Log ("App.Compiled\t%s@%s %s\n",
            AppStartInfo.App_CompileUser,
            AppStartInfo.App_CompileSystem,
            AppStartInfo.App_CompileTime);

    i = 0;
    Log ("App.CompileFlags:\n");
    while (AppStartInfo.App_CompileFlags != NULL
        && AppStartInfo.App_CompileFlags[i] != NULL) {
        Log ("			%s\n", AppStartInfo.App_CompileFlags[i++]);
    }

    i = 0;
    Log ("App.Libs:\n");
    while (AppStartInfo.App_Libs != NULL && AppStartInfo.App_Libs[i] != NULL)
        Log ("			%s\n", AppStartInfo.App_Libs[i++]);

    /* Security */
    i = 0;
    Log ("App.Suffixes:\n");
    while (AppStartInfo.Suffix.List != NULL && AppStartInfo.Suffix.List[i] != NULL)
        Log ("			%s\n", AppStartInfo.Suffix.List[i++]);

    i = 0;
    Log ("App.Keys:\n");
    while (AppStartInfo.Key.List != NULL && AppStartInfo.Key.List[i] != NULL)
        Log ("			%s\n", AppStartInfo.Key.List[i++]);


    /*** Linked libraries */
    Log ("App.Version.Driver =\t%s\n",  IPGDrv_LibVersion);
    Log ("App.Version.Road =\t%s\n",    RoadLibVersion);
}




int
App_ExportConfig (void)
{
    int        i, n;
    char       *txt[42], sbuf[512];
    char const *item;
    tInfos *inf = SimCore.Config.Inf;

    InfoSetStr (inf, "Application.Version",       AppStartInfo.App_Version);
    InfoSetStr (inf, "Application.BuildVersion",  AppStartInfo.App_BuildVersion);
    InfoSetStr (inf, "Application.CompileTime",   AppStartInfo.App_CompileTime);
    InfoSetStr (inf, "Application.CompileUser",   AppStartInfo.App_CompileUser);
    InfoSetStr (inf, "Application.CompileSystem", AppStartInfo.App_CompileSystem);
    if (AppStartInfo.App_CompileFlags != NULL)
        InfoSetTxt (inf, "Application.CompileFlags",
                    (char**)AppStartInfo.App_CompileFlags);
    InfoAddLineBehind (inf, NULL, "");
    if (AppStartInfo.App_Libs != NULL)
        InfoSetTxt (inf, "Application.Libs",
                    (char**)AppStartInfo.App_Libs);
    InfoAddLineBehind (inf, NULL, "");

    /*** Linked libraries */
    InfoSetStr (inf, "Application.Version.Driver",  IPGDrv_LibVersion);
    InfoSetStr (inf, "Application.Version.Road",    RoadLibVersion);
    InfoAddLineBehind (inf, NULL, "");

    /*** I/O configuration */
    IO_ListNames(sbuf, -1);

    n = 0;
    txt[n] = NULL;
    while (1) {
	item = strtok((n==0 ? sbuf : NULL), " \t");
	if (item == NULL || n >= 42-1)
	    break;

	txt[n++] = strdup(item);
	txt[n] =   NULL;
    }

    InfoSetTxt (inf, "IO.Configs", txt);
    InfoAddLineBehind (inf, NULL, "");

    for (i=0; i < n; i++)
	free (txt[i]);

    return 0;
}



