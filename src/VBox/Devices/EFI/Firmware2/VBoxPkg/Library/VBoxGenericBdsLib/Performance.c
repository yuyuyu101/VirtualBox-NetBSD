/* $Id: Performance.c $ */
/** @file
 * Performance.c - This file include the file which can help to get the
 * system performance, all the function will only include if the performance
 * switch is set.
 */

/*
 * Copyright (C) 2010 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

/*

This code is based on:

Copyright (c) 2004 - 2009, Intel Corporation. <BR>
All rights reserved. This program and the accompanying materials
are licensed and made available under the terms and conditions of the BSD License
which accompanies this distribution.  The full text of the license may be found at
http://opensource.org/licenses/bsd-license.php

THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

*/

#include "InternalBdsLib.h"

PERF_HEADER               mPerfHeader;
PERF_DATA                 mPerfData;
EFI_PHYSICAL_ADDRESS      mAcpiLowMemoryBase = 0x0FFFFFFFFULL;

/**
  Get the short version of PDB file name to be
  used in performance data logging.

  @param PdbFileName     The long PDB file name.
  @param GaugeString     The output string to be logged by performance logger.

**/
VOID
GetShortPdbFileName (
  IN  CONST CHAR8  *PdbFileName,
  OUT       CHAR8  *GaugeString
  )
{
  UINTN Index;
  UINTN Index1;
  UINTN StartIndex;
  UINTN EndIndex;

  if (PdbFileName == NULL) {
    AsciiStrCpy (GaugeString, " ");
  } else {
    StartIndex = 0;
    for (EndIndex = 0; PdbFileName[EndIndex] != 0; EndIndex++)
      ;

    for (Index = 0; PdbFileName[Index] != 0; Index++) {
      if (PdbFileName[Index] == '\\') {
        StartIndex = Index + 1;
      }

      if (PdbFileName[Index] == '.') {
        EndIndex = Index;
      }
    }

    Index1 = 0;
    for (Index = StartIndex; Index < EndIndex; Index++) {
      GaugeString[Index1] = PdbFileName[Index];
      Index1++;
      if (Index1 == PERF_TOKEN_LENGTH - 1) {
        break;
      }
    }

    GaugeString[Index1] = 0;
  }

  return ;
}

/**
  Get the name from the Driver handle, which can be a handle with
  EFI_LOADED_IMAGE_PROTOCOL or EFI_DRIVER_BINDING_PROTOCOL installed.
  This name can be used in performance data logging.

  @param Handle          Driver handle.
  @param GaugeString     The output string to be logged by performance logger.

**/
VOID
GetNameFromHandle (
  IN  EFI_HANDLE     Handle,
  OUT CHAR8          *GaugeString
  )
{
  EFI_STATUS                  Status;
  EFI_LOADED_IMAGE_PROTOCOL   *Image;
  CHAR8                       *PdbFileName;
  EFI_DRIVER_BINDING_PROTOCOL *DriverBinding;

  AsciiStrCpy (GaugeString, " ");

  //
  // Get handle name from image protocol
  //
  Status = gBS->HandleProtocol (
                  Handle,
                  &gEfiLoadedImageProtocolGuid,
                  (VOID **) &Image
                  );

  if (EFI_ERROR (Status)) {
    Status = gBS->OpenProtocol (
                    Handle,
                    &gEfiDriverBindingProtocolGuid,
                    (VOID **) &DriverBinding,
                    NULL,
                    NULL,
                    EFI_OPEN_PROTOCOL_GET_PROTOCOL
                    );
    if (EFI_ERROR (Status)) {
      return ;
    }
    //
    // Get handle name from image protocol
    //
    Status = gBS->HandleProtocol (
                    DriverBinding->ImageHandle,
                    &gEfiLoadedImageProtocolGuid,
                    (VOID **) &Image
                    );
  }

  PdbFileName = PeCoffLoaderGetPdbPointer (Image->ImageBase);

  if (PdbFileName != NULL) {
    GetShortPdbFileName (PdbFileName, GaugeString);
  }

  return ;
}

/**

  Allocates a block of memory and writes performance data of booting into it.
  OS can processing these record.

**/
VOID
WriteBootToOsPerformanceData (
  VOID
  )
{
  EFI_STATUS                Status;
  UINT32                    AcpiLowMemoryLength;
  UINT32                    LimitCount;
  EFI_HANDLE                *Handles;
  UINTN                     NoHandles;
  CHAR8                     GaugeString[PERF_TOKEN_LENGTH];
  UINT8                     *Ptr;
  UINT32                    Index;
  UINT64                    Ticker;
  UINT64                    Freq;
  UINT32                    Duration;
  UINTN                     LogEntryKey;
  CONST VOID                *Handle;
  CONST CHAR8               *Token;
  CONST CHAR8               *Module;
  UINT64                    StartTicker;
  UINT64                    EndTicker;
  UINT64                    StartValue;
  UINT64                    EndValue;
  BOOLEAN                   CountUp;

  //
  // Retrieve time stamp count as early as possible
  //
  Ticker  = GetPerformanceCounter ();

  Freq    = GetPerformanceCounterProperties (&StartValue, &EndValue);

  Freq    = DivU64x32 (Freq, 1000);

  mPerfHeader.CpuFreq = Freq;

  //
  // Record BDS raw performance data
  //
  if (EndValue >= StartValue) {
    mPerfHeader.BDSRaw = Ticker - StartValue;
    CountUp            = TRUE;
  } else {
    mPerfHeader.BDSRaw = StartValue - Ticker;
    CountUp            = FALSE;
  }

  //
  // Put Detailed performance data into memory
  //
  Handles = NULL;
  Status = gBS->LocateHandleBuffer (
                  AllHandles,
                  NULL,
                  NULL,
                  &NoHandles,
                  &Handles
                  );
  if (EFI_ERROR (Status)) {
    return ;
  }


  AcpiLowMemoryLength = 0x4000;
  if (mAcpiLowMemoryBase == 0x0FFFFFFFF) {
    //
    // Allocate a block of memory that contain performance data to OS
    //
    Status = gBS->AllocatePages (
                    AllocateMaxAddress,
                    EfiReservedMemoryType,
                    EFI_SIZE_TO_PAGES (AcpiLowMemoryLength),
                    &mAcpiLowMemoryBase
                    );
    if (EFI_ERROR (Status)) {
      FreePool (Handles);
      return ;
    }
  }


  Ptr        = (UINT8 *) ((UINT32) mAcpiLowMemoryBase + sizeof (PERF_HEADER));
  LimitCount = (AcpiLowMemoryLength - sizeof (PERF_HEADER)) / sizeof (PERF_DATA);



  //
  // Get DXE drivers performance
  //
  for (Index = 0; Index < NoHandles; Index++) {
    Ticker = 0;
    LogEntryKey = 0;
    while ((LogEntryKey = GetPerformanceMeasurement (
                            LogEntryKey,
                            &Handle,
                            &Token,
                            &Module,
                            &StartTicker,
                            &EndTicker)) != 0) {
      if ((Handle == Handles[Index]) && (EndTicker != 0)) {
        Ticker += CountUp ? (EndTicker - StartTicker) : (StartTicker - EndTicker);
      }
    }

    Duration = (UINT32) DivU64x32 (Ticker, (UINT32) Freq);

    if (Duration > 0) {

      GetNameFromHandle (Handles[Index], GaugeString);

      AsciiStrCpy (mPerfData.Token, GaugeString);
      mPerfData.Duration = Duration;

      CopyMem (Ptr, &mPerfData, sizeof (PERF_DATA));
      Ptr += sizeof (PERF_DATA);

      mPerfHeader.Count++;
      if (mPerfHeader.Count == LimitCount) {
        goto Done;
      }
    }
  }

  FreePool (Handles);

  //
  // Get inserted performance data
  //
  LogEntryKey = 0;
  while ((LogEntryKey = GetPerformanceMeasurement (
                          LogEntryKey,
                          &Handle,
                          &Token,
                          &Module,
                          &StartTicker,
                          &EndTicker)) != 0) {
    if (Handle == NULL && EndTicker != 0) {

      ZeroMem (&mPerfData, sizeof (PERF_DATA));

      AsciiStrnCpy (mPerfData.Token, Token, PERF_TOKEN_LENGTH);
      Ticker = CountUp ? (EndTicker - StartTicker) : (StartTicker - EndTicker);

      mPerfData.Duration = (UINT32) DivU64x32 (Ticker, (UINT32) Freq);

      CopyMem (Ptr, &mPerfData, sizeof (PERF_DATA));
      Ptr += sizeof (PERF_DATA);

      mPerfHeader.Count++;
      if (mPerfHeader.Count == LimitCount) {
        goto Done;
      }
    }
  }

Done:

  mPerfHeader.Signiture = PERFORMANCE_SIGNATURE;

  //
  // Put performance data to Reserved memory
  //
  CopyMem (
    (UINTN *) (UINTN) mAcpiLowMemoryBase,
    &mPerfHeader,
    sizeof (PERF_HEADER)
    );

  gRT->SetVariable (
        L"PerfDataMemAddr",
        &gPerformanceProtocolGuid,
        EFI_VARIABLE_NON_VOLATILE | EFI_VARIABLE_BOOTSERVICE_ACCESS | EFI_VARIABLE_RUNTIME_ACCESS,
        sizeof (EFI_PHYSICAL_ADDRESS),
        &mAcpiLowMemoryBase
        );

  return ;
}
