/* $Id: MachineImpl.h $ */
/** @file
 * Implementation of IMachine in VBoxSVC - Header.
 */

/*
 * Copyright (C) 2006-2013 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#ifndef ____H_MACHINEIMPL
#define ____H_MACHINEIMPL

#include "VirtualBoxBase.h"
#include "SnapshotImpl.h"
#include "ProgressImpl.h"
#include "VRDEServerImpl.h"
#include "MediumAttachmentImpl.h"
#include "PCIDeviceAttachmentImpl.h"
#include "MediumLock.h"
#include "NetworkAdapterImpl.h"
#include "AudioAdapterImpl.h"
#include "SerialPortImpl.h"
#include "ParallelPortImpl.h"
#include "BIOSSettingsImpl.h"
#include "StorageControllerImpl.h"          // required for MachineImpl.h to compile on Windows
#include "BandwidthControlImpl.h"
#include "BandwidthGroupImpl.h"
#include "VBox/settings.h"
#ifdef VBOX_WITH_RESOURCE_USAGE_API
#include "Performance.h"
#include "PerformanceImpl.h"
#endif /* VBOX_WITH_RESOURCE_USAGE_API */

// generated header
#include "SchemaDefs.h"

#include "VBox/com/ErrorInfo.h"

#include <iprt/file.h>
#include <iprt/thread.h>
#include <iprt/time.h>

#include <list>
#include <vector>

// defines
////////////////////////////////////////////////////////////////////////////////

// helper declarations
////////////////////////////////////////////////////////////////////////////////

class Progress;
class ProgressProxy;
class Keyboard;
class Mouse;
class Display;
class MachineDebugger;
class USBController;
class Snapshot;
class SharedFolder;
class HostUSBDevice;
class StorageController;

class SessionMachine;

namespace settings
{
    class MachineConfigFile;
    struct Snapshot;
    struct Hardware;
    struct Storage;
    struct StorageController;
    struct MachineRegistryEntry;
}

// Machine class
////////////////////////////////////////////////////////////////////////////////

class ATL_NO_VTABLE Machine :
    public VirtualBoxBase,
    VBOX_SCRIPTABLE_IMPL(IMachine)
{
    Q_OBJECT

public:

    enum StateDependency
    {
        AnyStateDep = 0,
        MutableStateDep,
        MutableOrSavedStateDep,
        OfflineStateDep
    };

    /**
     * Internal machine data.
     *
     * Only one instance of this data exists per every machine -- it is shared
     * by the Machine, SessionMachine and all SnapshotMachine instances
     * associated with the given machine using the util::Shareable template
     * through the mData variable.
     *
     * @note |const| members are persistent during lifetime so can be
     * accessed without locking.
     *
     * @note There is no need to lock anything inside init() or uninit()
     * methods, because they are always serialized (see AutoCaller).
     */
    struct Data
    {
        /**
         * Data structure to hold information about sessions opened for the
         * given machine.
         */
        struct Session
        {
            /** Control of the direct session opened by lockMachine() */
            ComPtr<IInternalSessionControl> mDirectControl;

            typedef std::list<ComPtr<IInternalSessionControl> > RemoteControlList;

            /** list of controls of all opened remote sessions */
            RemoteControlList mRemoteControls;

            /** launchVMProcess() and OnSessionEnd() progress indicator */
            ComObjPtr<ProgressProxy> mProgress;

            /**
             * PID of the session object that must be passed to openSession()
             * to finalize the launchVMProcess() request (i.e., PID of the
             * process created by launchVMProcess())
             */
            RTPROCESS mPID;

            /** Current session state */
            SessionState_T mState;

            /** Session type string (for indirect sessions) */
            Bstr mType;

            /** Session machine object */
            ComObjPtr<SessionMachine> mMachine;

            /** Medium object lock collection. */
            MediumLockListMap mLockedMedia;
        };

        Data();
        ~Data();

        const Guid          mUuid;
        BOOL                mRegistered;

        Utf8Str             m_strConfigFile;
        Utf8Str             m_strConfigFileFull;

        // machine settings XML file
        settings::MachineConfigFile *pMachineConfigFile;
        uint32_t            flModifications;
        bool                m_fAllowStateModification;

        BOOL                mAccessible;
        com::ErrorInfo      mAccessError;

        MachineState_T      mMachineState;
        RTTIMESPEC          mLastStateChange;

        /* Note: These are guarded by VirtualBoxBase::stateLockHandle() */
        uint32_t            mMachineStateDeps;
        RTSEMEVENTMULTI     mMachineStateDepsSem;
        uint32_t            mMachineStateChangePending;

        BOOL                mCurrentStateModified;
        /** Guest properties have been modified and need saving since the
         * machine was started, or there are transient properties which need
         * deleting and the machine is being shut down. */
        BOOL                mGuestPropertiesModified;

        Session             mSession;

        ComObjPtr<Snapshot> mFirstSnapshot;
        ComObjPtr<Snapshot> mCurrentSnapshot;

        // list of files to delete in Delete(); this list is filled by Unregister()
        std::list<Utf8Str>  llFilesToDelete;
    };

    /**
     *  Saved state data.
     *
     *  It's actually only the state file path string, but it needs to be
     *  separate from Data, because Machine and SessionMachine instances
     *  share it, while SnapshotMachine does not.
     *
     *  The data variable is |mSSData|.
     */
    struct SSData
    {
        Utf8Str strStateFilePath;
    };

    /**
     *  User changeable machine data.
     *
     *  This data is common for all machine snapshots, i.e. it is shared
     *  by all SnapshotMachine instances associated with the given machine
     *  using the util::Backupable template through the |mUserData| variable.
     *
     *  SessionMachine instances can alter this data and discard changes.
     *
     *  @note There is no need to lock anything inside init() or uninit()
     *  methods, because they are always serialized (see AutoCaller).
     */
    struct UserData
    {
        settings::MachineUserData s;
    };

    /**
     *  Hardware data.
     *
     *  This data is unique for a machine and for every machine snapshot.
     *  Stored using the util::Backupable template in the |mHWData| variable.
     *
     *  SessionMachine instances can alter this data and discard changes.
     */
    struct HWData
    {
        /**
         * Data structure to hold information about a guest property.
         */
        struct GuestProperty {
            /** Property value */
            Utf8Str strValue;
            /** Property timestamp */
            LONG64 mTimestamp;
            /** Property flags */
            ULONG mFlags;
        };

        HWData();
        ~HWData();

        Bstr                 mHWVersion;
        Guid                 mHardwareUUID;   /**< If Null, use mData.mUuid. */
        ULONG                mMemorySize;
        ULONG                mMemoryBalloonSize;
        BOOL                 mPageFusionEnabled;
        ULONG                mVRAMSize;
        ULONG                mVideoCaptureWidth;
        ULONG                mVideoCaptureHeight;
        Bstr                 mVideoCaptureFile;
        BOOL                 mVideoCaptureEnabled;
        ULONG                mMonitorCount;
        BOOL                 mHWVirtExEnabled;
        BOOL                 mHWVirtExExclusive;
        BOOL                 mHWVirtExNestedPagingEnabled;
        BOOL                 mHWVirtExLargePagesEnabled;
        BOOL                 mHWVirtExVPIDEnabled;
        BOOL                 mHWVirtExForceEnabled;
        BOOL                 mAccelerate2DVideoEnabled;
        BOOL                 mPAEEnabled;
        BOOL                 mSyntheticCpu;
        ULONG                mCPUCount;
        BOOL                 mCPUHotPlugEnabled;
        ULONG                mCpuExecutionCap;
        BOOL                 mAccelerate3DEnabled;
        BOOL                 mHPETEnabled;

        BOOL                 mCPUAttached[SchemaDefs::MaxCPUCount];

        settings::CpuIdLeaf  mCpuIdStdLeafs[11];
        settings::CpuIdLeaf  mCpuIdExtLeafs[11];

        DeviceType_T         mBootOrder[SchemaDefs::MaxBootPosition];

        typedef std::list<ComObjPtr<SharedFolder> > SharedFolderList;
        SharedFolderList     mSharedFolders;

        ClipboardMode_T      mClipboardMode;
        DragAndDropMode_T    mDragAndDropMode;

        typedef std::map<Utf8Str, GuestProperty> GuestPropertyMap;
        GuestPropertyMap     mGuestProperties;
        Utf8Str              mGuestPropertyNotificationPatterns;

        FirmwareType_T       mFirmwareType;
        KeyboardHIDType_T    mKeyboardHIDType;
        PointingHIDType_T    mPointingHIDType;
        ChipsetType_T        mChipsetType;
        BOOL                 mEmulatedUSBWebcamEnabled;
        BOOL                 mEmulatedUSBCardReaderEnabled;

        BOOL                 mIOCacheEnabled;
        ULONG                mIOCacheSize;

        typedef std::list<ComObjPtr<PCIDeviceAttachment> > PCIDeviceAssignmentList;
        PCIDeviceAssignmentList mPCIDeviceAssignments;

        settings::Debugging  mDebugging;
        settings::Autostart  mAutostart;
    };

    /**
   *  Hard disk and other media data.
     *
     *  The usage policy is the same as for HWData, but a separate structure
     *  is necessary because hard disk data requires different procedures when
     *  taking or deleting snapshots, etc.
     *
     *  The data variable is |mMediaData|.
     */
    struct MediaData
    {
        MediaData();
        ~MediaData();

        typedef std::list<ComObjPtr<MediumAttachment> > AttachmentList;
        AttachmentList mAttachments;
    };

    VIRTUALBOXBASE_ADD_ERRORINFO_SUPPORT(Machine, IMachine)

    DECLARE_NOT_AGGREGATABLE(Machine)

    DECLARE_PROTECT_FINAL_CONSTRUCT()

    BEGIN_COM_MAP(Machine)
        VBOX_DEFAULT_INTERFACE_ENTRIES(IMachine)
    END_COM_MAP()

    DECLARE_EMPTY_CTOR_DTOR(Machine)

    HRESULT FinalConstruct();
    void FinalRelease();

    // public initializer/uninitializer for internal purposes only:

    // initializer for creating a new, empty machine
    HRESULT init(VirtualBox *aParent,
                 const Utf8Str &strConfigFile,
                 const Utf8Str &strName,
                 const StringsList &llGroups,
                 GuestOSType *aOsType,
                 const Guid &aId,
                 bool fForceOverwrite,
                 bool fDirectoryIncludesUUID);

    // initializer for loading existing machine XML (either registered or not)
    HRESULT initFromSettings(VirtualBox *aParent,
                             const Utf8Str &strConfigFile,
                             const Guid *aId);

    // initializer for machine config in memory (OVF import)
    HRESULT init(VirtualBox *aParent,
                 const Utf8Str &strName,
                 const settings::MachineConfigFile &config);

    void uninit();

#ifdef VBOX_WITH_RESOURCE_USAGE_API
    // Needed from VirtualBox, for the delayed metrics cleanup.
    void unregisterMetrics(PerformanceCollector *aCollector, Machine *aMachine);
#endif /* VBOX_WITH_RESOURCE_USAGE_API */

protected:
    HRESULT initImpl(VirtualBox *aParent,
                     const Utf8Str &strConfigFile);
    HRESULT initDataAndChildObjects();
    HRESULT registeredInit();
    HRESULT tryCreateMachineConfigFile(bool fForceOverwrite);
    void uninitDataAndChildObjects();

public:
    // IMachine properties
    STDMETHOD(COMGETTER(Parent))(IVirtualBox **aParent);
    STDMETHOD(COMGETTER(Accessible))(BOOL *aAccessible);
    STDMETHOD(COMGETTER(AccessError))(IVirtualBoxErrorInfo **aAccessError);
    STDMETHOD(COMGETTER(Name))(BSTR *aName);
    STDMETHOD(COMSETTER(Name))(IN_BSTR aName);
    STDMETHOD(COMGETTER(Description))(BSTR *aDescription);
    STDMETHOD(COMSETTER(Description))(IN_BSTR aDescription);
    STDMETHOD(COMGETTER(Id))(BSTR *aId);
    STDMETHOD(COMGETTER(Groups))(ComSafeArrayOut(BSTR, aGroups));
    STDMETHOD(COMSETTER(Groups))(ComSafeArrayIn(IN_BSTR, aGroups));
    STDMETHOD(COMGETTER(OSTypeId))(BSTR *aOSTypeId);
    STDMETHOD(COMSETTER(OSTypeId))(IN_BSTR aOSTypeId);
    STDMETHOD(COMGETTER(HardwareVersion))(BSTR *aVersion);
    STDMETHOD(COMSETTER(HardwareVersion))(IN_BSTR aVersion);
    STDMETHOD(COMGETTER(HardwareUUID))(BSTR *aUUID);
    STDMETHOD(COMSETTER(HardwareUUID))(IN_BSTR aUUID);
    STDMETHOD(COMGETTER(MemorySize))(ULONG *memorySize);
    STDMETHOD(COMSETTER(MemorySize))(ULONG memorySize);
    STDMETHOD(COMGETTER(CPUCount))(ULONG *cpuCount);
    STDMETHOD(COMSETTER(CPUCount))(ULONG cpuCount);
    STDMETHOD(COMGETTER(CPUHotPlugEnabled))(BOOL *enabled);
    STDMETHOD(COMSETTER(CPUHotPlugEnabled))(BOOL enabled);
    STDMETHOD(COMGETTER(CPUExecutionCap))(ULONG *aExecutionCap);
    STDMETHOD(COMSETTER(CPUExecutionCap))(ULONG aExecutionCap);
    STDMETHOD(COMGETTER(EmulatedUSBCardReaderEnabled))(BOOL *enabled);
    STDMETHOD(COMSETTER(EmulatedUSBCardReaderEnabled))(BOOL enabled);
    STDMETHOD(COMGETTER(EmulatedUSBWebcameraEnabled))(BOOL *enabled);
    STDMETHOD(COMSETTER(EmulatedUSBWebcameraEnabled))(BOOL enabled);
    STDMETHOD(COMGETTER(HPETEnabled))(BOOL *enabled);
    STDMETHOD(COMSETTER(HPETEnabled))(BOOL enabled);
    STDMETHOD(COMGETTER(MemoryBalloonSize))(ULONG *memoryBalloonSize);
    STDMETHOD(COMSETTER(MemoryBalloonSize))(ULONG memoryBalloonSize);
    STDMETHOD(COMGETTER(PageFusionEnabled))(BOOL *enabled);
    STDMETHOD(COMSETTER(PageFusionEnabled))(BOOL enabled);
    STDMETHOD(COMGETTER(VRAMSize))(ULONG *memorySize);
    STDMETHOD(COMSETTER(VRAMSize))(ULONG memorySize);
    STDMETHOD(COMGETTER(VideoCaptureEnabled))(BOOL *u8VideoRecEnabled);
    STDMETHOD(COMSETTER(VideoCaptureEnabled))(BOOL  u8VideoRecEnabled);
    STDMETHOD(COMGETTER(VideoCaptureFile))(BSTR * ppChVideoRecFilename);
    STDMETHOD(COMSETTER(VideoCaptureFile))(IN_BSTR pChVideoRecFilename);
    STDMETHOD(COMGETTER(VideoCaptureWidth))(ULONG *u32VideoRecHorzRes);
    STDMETHOD(COMSETTER(VideoCaptureWidth))(ULONG u32VideoRecHorzRes);
    STDMETHOD(COMGETTER(VideoCaptureHeight))(ULONG *u32VideoRecVertRes);
    STDMETHOD(COMSETTER(VideoCaptureHeight))(ULONG u32VideoRecVertRes);
    STDMETHOD(COMGETTER(MonitorCount))(ULONG *monitorCount);
    STDMETHOD(COMSETTER(MonitorCount))(ULONG monitorCount);
    STDMETHOD(COMGETTER(Accelerate3DEnabled))(BOOL *enabled);
    STDMETHOD(COMSETTER(Accelerate3DEnabled))(BOOL enabled);
    STDMETHOD(COMGETTER(Accelerate2DVideoEnabled))(BOOL *enabled);
    STDMETHOD(COMSETTER(Accelerate2DVideoEnabled))(BOOL enabled);
    STDMETHOD(COMGETTER(BIOSSettings))(IBIOSSettings **biosSettings);
    STDMETHOD(COMGETTER(SnapshotFolder))(BSTR *aSavedStateFolder);
    STDMETHOD(COMSETTER(SnapshotFolder))(IN_BSTR aSavedStateFolder);
    STDMETHOD(COMGETTER(MediumAttachments))(ComSafeArrayOut(IMediumAttachment *, aAttachments));
    STDMETHOD(COMGETTER(VRDEServer))(IVRDEServer **vrdeServer);
    STDMETHOD(COMGETTER(AudioAdapter))(IAudioAdapter **audioAdapter);
    STDMETHOD(COMGETTER(USBController))(IUSBController * *aUSBController);
    STDMETHOD(COMGETTER(SettingsFilePath))(BSTR *aFilePath);
    STDMETHOD(COMGETTER(SettingsModified))(BOOL *aModified);
    STDMETHOD(COMGETTER(SessionState))(SessionState_T *aSessionState);
    STDMETHOD(COMGETTER(SessionType))(BSTR *aSessionType);
    STDMETHOD(COMGETTER(SessionPID))(ULONG *aSessionPID);
    STDMETHOD(COMGETTER(State))(MachineState_T *machineState);
    STDMETHOD(COMGETTER(LastStateChange))(LONG64 *aLastStateChange);
    STDMETHOD(COMGETTER(StateFilePath))(BSTR *aStateFilePath);
    STDMETHOD(COMGETTER(LogFolder))(BSTR *aLogFolder);
    STDMETHOD(COMGETTER(CurrentSnapshot))(ISnapshot **aCurrentSnapshot);
    STDMETHOD(COMGETTER(SnapshotCount))(ULONG *aSnapshotCount);
    STDMETHOD(COMGETTER(CurrentStateModified))(BOOL *aCurrentStateModified);
    STDMETHOD(COMGETTER(SharedFolders))(ComSafeArrayOut(ISharedFolder *, aSharedFolders));
    STDMETHOD(COMGETTER(ClipboardMode))(ClipboardMode_T *aClipboardMode);
    STDMETHOD(COMSETTER(ClipboardMode))(ClipboardMode_T aClipboardMode);
    STDMETHOD(COMGETTER(DragAndDropMode))(DragAndDropMode_T *aDragAndDropMode);
    STDMETHOD(COMSETTER(DragAndDropMode))(DragAndDropMode_T aDragAndDropMode);
    STDMETHOD(COMGETTER(GuestPropertyNotificationPatterns))(BSTR *aPattern);
    STDMETHOD(COMSETTER(GuestPropertyNotificationPatterns))(IN_BSTR aPattern);
    STDMETHOD(COMGETTER(StorageControllers))(ComSafeArrayOut(IStorageController *, aStorageControllers));
    STDMETHOD(COMGETTER(TeleporterEnabled))(BOOL *aEnabled);
    STDMETHOD(COMSETTER(TeleporterEnabled))(BOOL aEnabled);
    STDMETHOD(COMGETTER(TeleporterPort))(ULONG *aPort);
    STDMETHOD(COMSETTER(TeleporterPort))(ULONG aPort);
    STDMETHOD(COMGETTER(TeleporterAddress))(BSTR *aAddress);
    STDMETHOD(COMSETTER(TeleporterAddress))(IN_BSTR aAddress);
    STDMETHOD(COMGETTER(TeleporterPassword))(BSTR *aPassword);
    STDMETHOD(COMSETTER(TeleporterPassword))(IN_BSTR aPassword);
    STDMETHOD(COMGETTER(FaultToleranceState))(FaultToleranceState_T *aEnabled);
    STDMETHOD(COMSETTER(FaultToleranceState))(FaultToleranceState_T aEnabled);
    STDMETHOD(COMGETTER(FaultToleranceAddress))(BSTR *aAddress);
    STDMETHOD(COMSETTER(FaultToleranceAddress))(IN_BSTR aAddress);
    STDMETHOD(COMGETTER(FaultTolerancePort))(ULONG *aPort);
    STDMETHOD(COMSETTER(FaultTolerancePort))(ULONG aPort);
    STDMETHOD(COMGETTER(FaultTolerancePassword))(BSTR *aPassword);
    STDMETHOD(COMSETTER(FaultTolerancePassword))(IN_BSTR aPassword);
    STDMETHOD(COMGETTER(FaultToleranceSyncInterval))(ULONG *aInterval);
    STDMETHOD(COMSETTER(FaultToleranceSyncInterval))(ULONG aInterval);
    STDMETHOD(COMGETTER(RTCUseUTC))(BOOL *aEnabled);
    STDMETHOD(COMSETTER(RTCUseUTC))(BOOL aEnabled);
    STDMETHOD(COMGETTER(FirmwareType))(FirmwareType_T *aFirmware);
    STDMETHOD(COMSETTER(FirmwareType))(FirmwareType_T  aFirmware);
    STDMETHOD(COMGETTER(KeyboardHIDType))(KeyboardHIDType_T *aKeyboardHIDType);
    STDMETHOD(COMSETTER(KeyboardHIDType))(KeyboardHIDType_T  aKeyboardHIDType);
    STDMETHOD(COMGETTER(PointingHIDType))(PointingHIDType_T *aPointingHIDType);
    STDMETHOD(COMSETTER(PointingHIDType))(PointingHIDType_T  aPointingHIDType);
    STDMETHOD(COMGETTER(ChipsetType))(ChipsetType_T *aChipsetType);
    STDMETHOD(COMSETTER(ChipsetType))(ChipsetType_T  aChipsetType);
    STDMETHOD(COMGETTER(IOCacheEnabled))(BOOL *aEnabled);
    STDMETHOD(COMSETTER(IOCacheEnabled))(BOOL  aEnabled);
    STDMETHOD(COMGETTER(IOCacheSize))(ULONG *aIOCacheSize);
    STDMETHOD(COMSETTER(IOCacheSize))(ULONG  aIOCacheSize);
    STDMETHOD(COMGETTER(PCIDeviceAssignments))(ComSafeArrayOut(IPCIDeviceAttachment *, aAssignments));
    STDMETHOD(COMGETTER(BandwidthControl))(IBandwidthControl **aBandwidthControl);
    STDMETHOD(COMGETTER(TracingEnabled))(BOOL *pfEnabled);
    STDMETHOD(COMSETTER(TracingEnabled))(BOOL fEnabled);
    STDMETHOD(COMGETTER(TracingConfig))(BSTR *pbstrConfig);
    STDMETHOD(COMSETTER(TracingConfig))(IN_BSTR bstrConfig);
    STDMETHOD(COMGETTER(AllowTracingToAccessVM))(BOOL *pfAllow);
    STDMETHOD(COMSETTER(AllowTracingToAccessVM))(BOOL fAllow);
    STDMETHOD(COMGETTER(AutostartEnabled))(BOOL *pfEnabled);
    STDMETHOD(COMSETTER(AutostartEnabled))(BOOL fEnabled);
    STDMETHOD(COMGETTER(AutostartDelay))(ULONG *puDelay);
    STDMETHOD(COMSETTER(AutostartDelay))(ULONG uDelay);
    STDMETHOD(COMGETTER(AutostopType))(AutostopType_T *penmAutostopType);
    STDMETHOD(COMSETTER(AutostopType))(AutostopType_T enmAutostopType);

    // IMachine methods
    STDMETHOD(LockMachine)(ISession *aSession, LockType_T lockType);
    STDMETHOD(LaunchVMProcess)(ISession *aSession,  IN_BSTR aType, IN_BSTR aEnvironment, IProgress **aProgress);

    STDMETHOD(SetBootOrder)(ULONG aPosition, DeviceType_T aDevice);
    STDMETHOD(GetBootOrder)(ULONG aPosition, DeviceType_T *aDevice);
    STDMETHOD(AttachDeviceWithoutMedium)(IN_BSTR aControllerName, LONG aControllerPort,
                                        LONG aDevice, DeviceType_T aType);
    STDMETHOD(AttachDevice)(IN_BSTR aControllerName, LONG aControllerPort,
                            LONG aDevice, DeviceType_T aType, IMedium *aMedium);
    STDMETHOD(DetachDevice)(IN_BSTR aControllerName, LONG aControllerPort, LONG aDevice);
    STDMETHOD(PassthroughDevice)(IN_BSTR aControllerName, LONG aControllerPort, LONG aDevice, BOOL aPassthrough);
    STDMETHOD(TemporaryEjectDevice)(IN_BSTR aControllerName, LONG aControllerPort, LONG aDevice, BOOL aTempEject);
    STDMETHOD(NonRotationalDevice)(IN_BSTR aControllerName, LONG aControllerPort, LONG aDevice, BOOL aNonRotational);
    STDMETHOD(SetAutoDiscardForDevice)(IN_BSTR aControllerName, LONG aControllerPort, LONG aDevice, BOOL aDiscard);
    STDMETHOD(SetNoBandwidthGroupForDevice)(IN_BSTR aControllerName, LONG aControllerPort,
                                            LONG aDevice);
    STDMETHOD(SetBandwidthGroupForDevice)(IN_BSTR aControllerName, LONG aControllerPort,
                                          LONG aDevice, IBandwidthGroup *aBandwidthGroup);
    STDMETHOD(MountMedium)(IN_BSTR aControllerName, LONG aControllerPort,
                           LONG aDevice, IMedium *aMedium, BOOL aForce);
    STDMETHOD(UnmountMedium)(IN_BSTR aControllerName, LONG aControllerPort,
                             LONG aDevice, BOOL aForce);
    STDMETHOD(GetMedium)(IN_BSTR aControllerName, LONG aControllerPort, LONG aDevice,
                         IMedium **aMedium);
    STDMETHOD(GetSerialPort)(ULONG slot, ISerialPort **port);
    STDMETHOD(GetParallelPort)(ULONG slot, IParallelPort **port);
    STDMETHOD(GetNetworkAdapter)(ULONG slot, INetworkAdapter **adapter);
    STDMETHOD(GetExtraDataKeys)(ComSafeArrayOut(BSTR, aKeys));
    STDMETHOD(GetExtraData)(IN_BSTR aKey, BSTR *aValue);
    STDMETHOD(SetExtraData)(IN_BSTR aKey, IN_BSTR aValue);
    STDMETHOD(GetCPUProperty)(CPUPropertyType_T property, BOOL *aVal);
    STDMETHOD(SetCPUProperty)(CPUPropertyType_T property, BOOL aVal);
    STDMETHOD(GetCPUIDLeaf)(ULONG id, ULONG *aValEax, ULONG *aValEbx, ULONG *aValEcx, ULONG *aValEdx);
    STDMETHOD(SetCPUIDLeaf)(ULONG id, ULONG aValEax, ULONG aValEbx, ULONG aValEcx, ULONG aValEdx);
    STDMETHOD(RemoveCPUIDLeaf)(ULONG id);
    STDMETHOD(RemoveAllCPUIDLeaves)();
    STDMETHOD(GetHWVirtExProperty)(HWVirtExPropertyType_T property, BOOL *aVal);
    STDMETHOD(SetHWVirtExProperty)(HWVirtExPropertyType_T property, BOOL aVal);
    STDMETHOD(SaveSettings)();
    STDMETHOD(DiscardSettings)();
    STDMETHOD(Unregister)(CleanupMode_T cleanupMode, ComSafeArrayOut(IMedium*, aMedia));
    STDMETHOD(Delete)(ComSafeArrayIn(IMedium*, aMedia), IProgress **aProgress);
    STDMETHOD(Export)(IAppliance *aAppliance, IN_BSTR location, IVirtualSystemDescription **aDescription);
    STDMETHOD(FindSnapshot)(IN_BSTR aNameOrId, ISnapshot **aSnapshot);
    STDMETHOD(CreateSharedFolder)(IN_BSTR aName, IN_BSTR aHostPath, BOOL aWritable, BOOL aAutoMount);
    STDMETHOD(RemoveSharedFolder)(IN_BSTR aName);
    STDMETHOD(CanShowConsoleWindow)(BOOL *aCanShow);
    STDMETHOD(ShowConsoleWindow)(LONG64 *aWinId);
    STDMETHOD(GetGuestProperty)(IN_BSTR aName, BSTR *aValue, LONG64 *aTimestamp, BSTR *aFlags);
    STDMETHOD(GetGuestPropertyValue)(IN_BSTR aName, BSTR *aValue);
    STDMETHOD(GetGuestPropertyTimestamp)(IN_BSTR aName, LONG64 *aTimestamp);
    STDMETHOD(SetGuestProperty)(IN_BSTR aName, IN_BSTR aValue, IN_BSTR aFlags);
    STDMETHOD(SetGuestPropertyValue)(IN_BSTR aName, IN_BSTR aValue);
    STDMETHOD(DeleteGuestProperty)(IN_BSTR aName);
    STDMETHOD(EnumerateGuestProperties)(IN_BSTR aPattern, ComSafeArrayOut(BSTR, aNames), ComSafeArrayOut(BSTR, aValues), ComSafeArrayOut(LONG64, aTimestamps), ComSafeArrayOut(BSTR, aFlags));
    STDMETHOD(GetMediumAttachmentsOfController)(IN_BSTR aName, ComSafeArrayOut(IMediumAttachment *, aAttachments));
    STDMETHOD(GetMediumAttachment)(IN_BSTR aConstrollerName, LONG aControllerPort, LONG aDevice, IMediumAttachment **aAttachment);
    STDMETHOD(AddStorageController)(IN_BSTR aName, StorageBus_T aConnectionType, IStorageController **controller);
    STDMETHOD(RemoveStorageController(IN_BSTR aName));
    STDMETHOD(GetStorageControllerByName(IN_BSTR aName, IStorageController **storageController));
    STDMETHOD(GetStorageControllerByInstance(ULONG aInstance, IStorageController **storageController));
    STDMETHOD(SetStorageControllerBootable)(IN_BSTR aName, BOOL fBootable);
    STDMETHOD(QuerySavedGuestScreenInfo)(ULONG uScreenId, ULONG *puOriginX, ULONG *puOriginY, ULONG *puWidth, ULONG *puHeight, BOOL *pfEnabled);
    STDMETHOD(QuerySavedThumbnailSize)(ULONG aScreenId, ULONG *aSize, ULONG *aWidth, ULONG *aHeight);
    STDMETHOD(ReadSavedThumbnailToArray)(ULONG aScreenId, BOOL aBGR, ULONG *aWidth, ULONG *aHeight, ComSafeArrayOut(BYTE, aData));
    STDMETHOD(ReadSavedThumbnailPNGToArray)(ULONG aScreenId, ULONG *aWidth, ULONG *aHeight, ComSafeArrayOut(BYTE, aData));
    STDMETHOD(QuerySavedScreenshotPNGSize)(ULONG aScreenId, ULONG *aSize, ULONG *aWidth, ULONG *aHeight);
    STDMETHOD(ReadSavedScreenshotPNGToArray)(ULONG aScreenId, ULONG *aWidth, ULONG *aHeight, ComSafeArrayOut(BYTE, aData));
    STDMETHOD(HotPlugCPU(ULONG aCpu));
    STDMETHOD(HotUnplugCPU(ULONG aCpu));
    STDMETHOD(GetCPUStatus(ULONG aCpu, BOOL *aCpuAttached));
    STDMETHOD(QueryLogFilename(ULONG aIdx, BSTR *aName));
    STDMETHOD(ReadLog(ULONG aIdx, LONG64 aOffset, LONG64 aSize, ComSafeArrayOut(BYTE, aData)));
    STDMETHOD(AttachHostPCIDevice(LONG hostAddress, LONG desiredGuestAddress, BOOL tryToUnbind));
    STDMETHOD(DetachHostPCIDevice(LONG hostAddress));
    STDMETHOD(CloneTo(IMachine *pTarget, CloneMode_T mode, ComSafeArrayIn(CloneOptions_T, options), IProgress **pProgress));
    // public methods only for internal purposes

    virtual bool isSnapshotMachine() const
    {
        return false;
    }

    virtual bool isSessionMachine() const
    {
        return false;
    }

    /**
     * Override of the default locking class to be used for validating lock
     * order with the standard member lock handle.
     */
    virtual VBoxLockingClass getLockingClass() const
    {
        return LOCKCLASS_MACHINEOBJECT;
    }

    /// @todo (dmik) add lock and make non-inlined after revising classes
    //  that use it. Note: they should enter Machine lock to keep the returned
    //  information valid!
    bool isRegistered() { return !!mData->mRegistered; }

    // unsafe inline public methods for internal purposes only (ensure there is
    // a caller and a read lock before calling them!)

    /**
     * Returns the VirtualBox object this machine belongs to.
     *
     * @note This method doesn't check this object's readiness. Intended to be
     * used by ready Machine children (whose readiness is bound to the parent's
     * one) or after doing addCaller() manually.
     */
    VirtualBox* getVirtualBox() const { return mParent; }

    /**
     * Checks if this machine is accessible, without attempting to load the
     * config file.
     *
     * @note This method doesn't check this object's readiness. Intended to be
     * used by ready Machine children (whose readiness is bound to the parent's
     * one) or after doing addCaller() manually.
     */
    bool isAccessible() const { return !!mData->mAccessible; }

    /**
     * Returns this machine ID.
     *
     * @note This method doesn't check this object's readiness. Intended to be
     * used by ready Machine children (whose readiness is bound to the parent's
     * one) or after adding a caller manually.
     */
    const Guid& getId() const { return mData->mUuid; }

    /**
     * Returns the snapshot ID this machine represents or an empty UUID if this
     * instance is not SnapshotMachine.
     *
     * @note This method doesn't check this object's readiness. Intended to be
     * used by ready Machine children (whose readiness is bound to the parent's
     * one) or after adding a caller manually.
     */
    inline const Guid& getSnapshotId() const;

    /**
     * Returns this machine's full settings file path.
     *
     * @note This method doesn't lock this object or check its readiness.
     * Intended to be used only after doing addCaller() manually and locking it
     * for reading.
     */
    const Utf8Str& getSettingsFileFull() const { return mData->m_strConfigFileFull; }

    /**
     * Returns this machine name.
     *
     * @note This method doesn't lock this object or check its readiness.
     * Intended to be used only after doing addCaller() manually and locking it
     * for reading.
     */
    const Utf8Str& getName() const { return mUserData->s.strName; }

    enum
    {
        IsModified_MachineData          = 0x0001,
        IsModified_Storage              = 0x0002,
        IsModified_NetworkAdapters      = 0x0008,
        IsModified_SerialPorts          = 0x0010,
        IsModified_ParallelPorts        = 0x0020,
        IsModified_VRDEServer           = 0x0040,
        IsModified_AudioAdapter         = 0x0080,
        IsModified_USB                  = 0x0100,
        IsModified_BIOS                 = 0x0200,
        IsModified_SharedFolders        = 0x0400,
        IsModified_Snapshots            = 0x0800,
        IsModified_BandwidthControl     = 0x1000
    };

    /**
     * Returns various information about this machine.
     *
     * @note This method doesn't lock this object or check its readiness.
     * Intended to be used only after doing addCaller() manually and locking it
     * for reading.
     */
    ChipsetType_T getChipsetType() const { return mHWData->mChipsetType; }

    void setModified(uint32_t fl, bool fAllowStateModification = true);
    void setModifiedLock(uint32_t fl, bool fAllowStateModification = true);

    bool isStateModificationAllowed() const { return mData->m_fAllowStateModification; }
    void allowStateModification()           { mData->m_fAllowStateModification = true; }
    void disallowStateModification()        { mData->m_fAllowStateModification = false; }

    const StringsList &getGroups() const { return mUserData->s.llGroups; }

    // callback handlers
    virtual HRESULT onNetworkAdapterChange(INetworkAdapter * /* networkAdapter */, BOOL /* changeAdapter */) { return S_OK; }
    virtual HRESULT onNATRedirectRuleChange(ULONG /* slot */, BOOL /* fRemove */ , IN_BSTR /* name */,
                                 NATProtocol_T /* protocol */, IN_BSTR /* host ip */, LONG /* host port */, IN_BSTR /* guest port */, LONG /* guest port */ ) { return S_OK; }
    virtual HRESULT onSerialPortChange(ISerialPort * /* serialPort */) { return S_OK; }
    virtual HRESULT onParallelPortChange(IParallelPort * /* parallelPort */) { return S_OK; }
    virtual HRESULT onVRDEServerChange(BOOL /* aRestart */) { return S_OK; }
    virtual HRESULT onUSBControllerChange() { return S_OK; }
    virtual HRESULT onStorageControllerChange() { return S_OK; }
    virtual HRESULT onCPUChange(ULONG /* aCPU */, BOOL /* aRemove */) { return S_OK; }
    virtual HRESULT onCPUExecutionCapChange(ULONG /* aExecutionCap */) { return S_OK; }
    virtual HRESULT onMediumChange(IMediumAttachment * /* mediumAttachment */, BOOL /* force */) { return S_OK; }
    virtual HRESULT onSharedFolderChange() { return S_OK; }
    virtual HRESULT onClipboardModeChange(ClipboardMode_T /* aClipboardMode */) { return S_OK; }
    virtual HRESULT onDragAndDropModeChange(DragAndDropMode_T /* aDragAndDropMode */) { return S_OK; }
    virtual HRESULT onBandwidthGroupChange(IBandwidthGroup * /* aBandwidthGroup */) { return S_OK; }
    virtual HRESULT onStorageDeviceChange(IMediumAttachment * /* mediumAttachment */, BOOL /* remove */) { return S_OK; }

    HRESULT saveRegistryEntry(settings::MachineRegistryEntry &data);

    int calculateFullPath(const Utf8Str &strPath, Utf8Str &aResult);
    void copyPathRelativeToMachine(const Utf8Str &strSource, Utf8Str &strTarget);

    void getLogFolder(Utf8Str &aLogFolder);
    Utf8Str queryLogFilename(ULONG idx);

    void composeSavedStateFilename(Utf8Str &strStateFilePath);

    HRESULT launchVMProcess(IInternalSessionControl *aControl,
                            const Utf8Str &strType,
                            const Utf8Str &strEnvironment,
                            ProgressProxy *aProgress);

    HRESULT getDirectControl(ComPtr<IInternalSessionControl> *directControl)
    {
        HRESULT rc;
        *directControl = mData->mSession.mDirectControl;

        if (!*directControl)
            rc = E_ACCESSDENIED;
        else
            rc = S_OK;

        return rc;
    }

#if defined(RT_OS_WINDOWS)

    bool isSessionOpen(ComObjPtr<SessionMachine> &aMachine,
                       ComPtr<IInternalSessionControl> *aControl = NULL,
                       HANDLE *aIPCSem = NULL, bool aAllowClosing = false);
    bool isSessionSpawning(RTPROCESS *aPID = NULL);

    bool isSessionOpenOrClosing(ComObjPtr<SessionMachine> &aMachine,
                                ComPtr<IInternalSessionControl> *aControl = NULL,
                                HANDLE *aIPCSem = NULL)
    { return isSessionOpen(aMachine, aControl, aIPCSem, true /* aAllowClosing */); }

#elif defined(RT_OS_OS2)

    bool isSessionOpen(ComObjPtr<SessionMachine> &aMachine,
                       ComPtr<IInternalSessionControl> *aControl = NULL,
                       HMTX *aIPCSem = NULL, bool aAllowClosing = false);

    bool isSessionSpawning(RTPROCESS *aPID = NULL);

    bool isSessionOpenOrClosing(ComObjPtr<SessionMachine> &aMachine,
                                 ComPtr<IInternalSessionControl> *aControl = NULL,
                                 HMTX *aIPCSem = NULL)
    { return isSessionOpen(aMachine, aControl, aIPCSem, true /* aAllowClosing */); }

#else

    bool isSessionOpen(ComObjPtr<SessionMachine> &aMachine,
                       ComPtr<IInternalSessionControl> *aControl = NULL,
                       bool aAllowClosing = false);
    bool isSessionSpawning();

    bool isSessionOpenOrClosing(ComObjPtr<SessionMachine> &aMachine,
                                ComPtr<IInternalSessionControl> *aControl = NULL)
    { return isSessionOpen(aMachine, aControl, true /* aAllowClosing */); }

#endif

    bool checkForSpawnFailure();

    HRESULT prepareRegister();

    HRESULT getSharedFolder(CBSTR aName,
                            ComObjPtr<SharedFolder> &aSharedFolder,
                            bool aSetError = false)
    {
        AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);
        return findSharedFolder(aName, aSharedFolder, aSetError);
    }

    HRESULT addStateDependency(StateDependency aDepType = AnyStateDep,
                               MachineState_T *aState = NULL,
                               BOOL *aRegistered = NULL);
    void releaseStateDependency();

    HRESULT getBandwidthGroup(const Utf8Str &strBandwidthGroup,
                              ComObjPtr<BandwidthGroup> &pBandwidthGroup,
                              bool fSetError = false)
    {
        return mBandwidthControl->getBandwidthGroupByName(strBandwidthGroup,
                                                          pBandwidthGroup,
                                                          fSetError);
    }

protected:

    HRESULT checkStateDependency(StateDependency aDepType);

    Machine *getMachine();

    void ensureNoStateDependencies();

    virtual HRESULT setMachineState(MachineState_T aMachineState);

    HRESULT findSharedFolder(const Utf8Str &aName,
                             ComObjPtr<SharedFolder> &aSharedFolder,
                             bool aSetError = false);

    HRESULT loadSettings(bool aRegistered);
    HRESULT loadMachineDataFromSettings(const settings::MachineConfigFile &config,
                                        const Guid *puuidRegistry);
    HRESULT loadSnapshot(const settings::Snapshot &data,
                         const Guid &aCurSnapshotId,
                         Snapshot *aParentSnapshot);
    HRESULT loadHardware(const settings::Hardware &data, const settings::Debugging *pDbg,
                         const settings::Autostart *pAutostart);
    HRESULT loadDebugging(const settings::Debugging *pDbg);
    HRESULT loadAutostart(const settings::Autostart *pAutostart);
    HRESULT loadStorageControllers(const settings::Storage &data,
                                   const Guid *puuidRegistry,
                                   const Guid *puuidSnapshot);
    HRESULT loadStorageDevices(StorageController *aStorageController,
                               const settings::StorageController &data,
                               const Guid *puuidRegistry,
                               const Guid *puuidSnapshot);

    HRESULT findSnapshotById(const Guid &aId,
                             ComObjPtr<Snapshot> &aSnapshot,
                             bool aSetError = false);
    HRESULT findSnapshotByName(const Utf8Str &strName,
                               ComObjPtr<Snapshot> &aSnapshot,
                               bool aSetError = false);

    HRESULT getStorageControllerByName(const Utf8Str &aName,
                                       ComObjPtr<StorageController> &aStorageController,
                                       bool aSetError = false);

    HRESULT getMediumAttachmentsOfController(CBSTR aName,
                                             MediaData::AttachmentList &aAttachments);

    enum
    {
        /* flags for #saveSettings() */
        SaveS_ResetCurStateModified = 0x01,
        SaveS_InformCallbacksAnyway = 0x02,
        SaveS_Force = 0x04,
        /* flags for #saveStateSettings() */
        SaveSTS_CurStateModified = 0x20,
        SaveSTS_StateFilePath = 0x40,
        SaveSTS_StateTimeStamp = 0x80
    };

    HRESULT prepareSaveSettings(bool *pfNeedsGlobalSaveSettings);
    HRESULT saveSettings(bool *pfNeedsGlobalSaveSettings, int aFlags = 0);

    void copyMachineDataToSettings(settings::MachineConfigFile &config);
    HRESULT saveAllSnapshots(settings::MachineConfigFile &config);
    HRESULT saveHardware(settings::Hardware &data, settings::Debugging *pDbg,
                         settings::Autostart *pAutostart);
    HRESULT saveStorageControllers(settings::Storage &data);
    HRESULT saveStorageDevices(ComObjPtr<StorageController> aStorageController,
                               settings::StorageController &data);
    HRESULT saveStateSettings(int aFlags);

    void addMediumToRegistry(ComObjPtr<Medium> &pMedium);

    HRESULT createImplicitDiffs(IProgress *aProgress,
                                ULONG aWeight,
                                bool aOnline);
    HRESULT deleteImplicitDiffs(bool aOnline);

    MediumAttachment* findAttachment(const MediaData::AttachmentList &ll,
                                     IN_BSTR aControllerName,
                                     LONG aControllerPort,
                                     LONG aDevice);
    MediumAttachment* findAttachment(const MediaData::AttachmentList &ll,
                                     ComObjPtr<Medium> pMedium);
    MediumAttachment* findAttachment(const MediaData::AttachmentList &ll,
                                     Guid &id);

    HRESULT detachDevice(MediumAttachment *pAttach,
                         AutoWriteLock &writeLock,
                         Snapshot *pSnapshot);

    HRESULT detachAllMedia(AutoWriteLock &writeLock,
                           Snapshot *pSnapshot,
                           CleanupMode_T cleanupMode,
                           MediaList &llMedia);

    void commitMedia(bool aOnline = false);
    void rollbackMedia();

    bool isInOwnDir(Utf8Str *aSettingsDir = NULL) const;

    void rollback(bool aNotify);
    void commit();
    void copyFrom(Machine *aThat);
    bool isControllerHotplugCapable(StorageControllerType_T enmCtrlType);

    struct DeleteTask;
    static DECLCALLBACK(int) deleteThread(RTTHREAD Thread, void *pvUser);
    HRESULT deleteTaskWorker(DeleteTask &task);

#ifdef VBOX_WITH_GUEST_PROPS
    HRESULT getGuestPropertyFromService(IN_BSTR aName, BSTR *aValue,
                                        LONG64 *aTimestamp, BSTR *aFlags) const;
    HRESULT getGuestPropertyFromVM(IN_BSTR aName, BSTR *aValue,
                                   LONG64 *aTimestamp, BSTR *aFlags) const;
    HRESULT setGuestPropertyToService(IN_BSTR aName, IN_BSTR aValue,
                                      IN_BSTR aFlags);
    HRESULT setGuestPropertyToVM(IN_BSTR aName, IN_BSTR aValue,
                                 IN_BSTR aFlags);
    HRESULT enumerateGuestPropertiesInService
                (IN_BSTR aPatterns, ComSafeArrayOut(BSTR, aNames),
                 ComSafeArrayOut(BSTR, aValues),
                 ComSafeArrayOut(LONG64, aTimestamps),
                 ComSafeArrayOut(BSTR, aFlags));
    HRESULT enumerateGuestPropertiesOnVM
                (IN_BSTR aPatterns, ComSafeArrayOut(BSTR, aNames),
                 ComSafeArrayOut(BSTR, aValues),
                 ComSafeArrayOut(LONG64, aTimestamps),
                 ComSafeArrayOut(BSTR, aFlags));
#endif /* VBOX_WITH_GUEST_PROPS */

#ifdef VBOX_WITH_RESOURCE_USAGE_API
    void getDiskList(MediaList &list);
    void registerMetrics(PerformanceCollector *aCollector, Machine *aMachine, RTPROCESS pid);

    pm::CollectorGuest     *mCollectorGuest;
#endif /* VBOX_WITH_RESOURCE_USAGE_API */

    Machine * const         mPeer;

    VirtualBox * const      mParent;

    Shareable<Data>         mData;
    Shareable<SSData>       mSSData;

    Backupable<UserData>    mUserData;
    Backupable<HWData>      mHWData;
    Backupable<MediaData>   mMediaData;

    // the following fields need special backup/rollback/commit handling,
    // so they cannot be a part of HWData

    const ComObjPtr<VRDEServer>     mVRDEServer;
    const ComObjPtr<SerialPort>     mSerialPorts[SchemaDefs::SerialPortCount];
    const ComObjPtr<ParallelPort>   mParallelPorts[SchemaDefs::ParallelPortCount];
    const ComObjPtr<AudioAdapter>   mAudioAdapter;
    const ComObjPtr<USBController>  mUSBController;
    const ComObjPtr<BIOSSettings>   mBIOSSettings;
    typedef std::vector<ComObjPtr<NetworkAdapter> > NetworkAdapterVector;
    NetworkAdapterVector            mNetworkAdapters;
    const ComObjPtr<BandwidthControl> mBandwidthControl;

    typedef std::list<ComObjPtr<StorageController> > StorageControllerList;
    Backupable<StorageControllerList> mStorageControllers;

    uint64_t                        uRegistryNeedsSaving;

    friend class SessionMachine;
    friend class SnapshotMachine;
    friend class Appliance;
    friend class VirtualBox;

    friend class MachineCloneVM;
};

// SessionMachine class
////////////////////////////////////////////////////////////////////////////////

/**
 *  @note Notes on locking objects of this class:
 *  SessionMachine shares some data with the primary Machine instance (pointed
 *  to by the |mPeer| member). In order to provide data consistency it also
 *  shares its lock handle. This means that whenever you lock a SessionMachine
 *  instance using Auto[Reader]Lock or AutoMultiLock, the corresponding Machine
 *  instance is also locked in the same lock mode. Keep it in mind.
 */
class ATL_NO_VTABLE SessionMachine :
    public Machine,
    VBOX_SCRIPTABLE_IMPL(IInternalMachineControl)
{
public:
    VIRTUALBOXBASE_ADD_ERRORINFO_SUPPORT(SessionMachine, IMachine)

    DECLARE_NOT_AGGREGATABLE(SessionMachine)

    DECLARE_PROTECT_FINAL_CONSTRUCT()

    BEGIN_COM_MAP(SessionMachine)
        VBOX_DEFAULT_INTERFACE_ENTRIES(IMachine)
        COM_INTERFACE_ENTRY(IInternalMachineControl)
    END_COM_MAP()

    DECLARE_EMPTY_CTOR_DTOR(SessionMachine)

    HRESULT FinalConstruct();
    void FinalRelease();

    // public initializer/uninitializer for internal purposes only
    HRESULT init(Machine *aMachine);
    void uninit() { uninit(Uninit::Unexpected); }

    // util::Lockable interface
    RWLockHandle *lockHandle() const;

    // IInternalMachineControl methods
    STDMETHOD(SetRemoveSavedStateFile)(BOOL aRemove);
    STDMETHOD(UpdateState)(MachineState_T machineState);
    STDMETHOD(GetIPCId)(BSTR *id);
    STDMETHOD(BeginPowerUp)(IProgress *aProgress);
    STDMETHOD(EndPowerUp)(LONG iResult);
    STDMETHOD(BeginPoweringDown)(IProgress **aProgress);
    STDMETHOD(EndPoweringDown)(LONG aResult, IN_BSTR aErrMsg);
    STDMETHOD(RunUSBDeviceFilters)(IUSBDevice *aUSBDevice, BOOL *aMatched, ULONG *aMaskedIfs);
    STDMETHOD(CaptureUSBDevice)(IN_BSTR aId);
    STDMETHOD(DetachUSBDevice)(IN_BSTR aId, BOOL aDone);
    STDMETHOD(AutoCaptureUSBDevices)();
    STDMETHOD(DetachAllUSBDevices)(BOOL aDone);
    STDMETHOD(OnSessionEnd)(ISession *aSession, IProgress **aProgress);
    STDMETHOD(BeginSavingState)(IProgress **aProgress, BSTR *aStateFilePath);
    STDMETHOD(EndSavingState)(LONG aResult, IN_BSTR aErrMsg);
    STDMETHOD(AdoptSavedState)(IN_BSTR aSavedStateFile);
    STDMETHOD(BeginTakingSnapshot)(IConsole *aInitiator,
                                   IN_BSTR aName,
                                   IN_BSTR aDescription,
                                   IProgress *aConsoleProgress,
                                   BOOL fTakingSnapshotOnline,
                                   BSTR *aStateFilePath);
    STDMETHOD(EndTakingSnapshot)(BOOL aSuccess);
    STDMETHOD(DeleteSnapshot)(IConsole *aInitiator, IN_BSTR aStartId,
                              IN_BSTR aEndID, BOOL fDeleteAllChildren,
                              MachineState_T *aMachineState, IProgress **aProgress);
    STDMETHOD(FinishOnlineMergeMedium)(IMediumAttachment *aMediumAttachment,
                                       IMedium *aSource, IMedium *aTarget,
                                       BOOL fMergeForward,
                                       IMedium *pParentForTarget,
                                       ComSafeArrayIn(IMedium *, aChildrenToReparent));
    STDMETHOD(RestoreSnapshot)(IConsole *aInitiator,
                               ISnapshot *aSnapshot,
                               MachineState_T *aMachineState,
                               IProgress **aProgress);
    STDMETHOD(PullGuestProperties)(ComSafeArrayOut(BSTR, aNames), ComSafeArrayOut(BSTR, aValues),
              ComSafeArrayOut(LONG64, aTimestamps), ComSafeArrayOut(BSTR, aFlags));
    STDMETHOD(PushGuestProperty)(IN_BSTR aName, IN_BSTR aValue,
                                  LONG64 aTimestamp, IN_BSTR aFlags);
    STDMETHOD(LockMedia)();
    STDMETHOD(UnlockMedia)();
    STDMETHOD(EjectMedium)(IMediumAttachment *aAttachment,
                           IMediumAttachment **aNewAttachment);
    STDMETHOD(ReportVmStatistics)(ULONG aValidStats, ULONG aCpuUser,
                                  ULONG aCpuKernel, ULONG aCpuIdle,
                                  ULONG aMemTotal, ULONG aMemFree,
                                  ULONG aMemBalloon, ULONG aMemShared,
                                  ULONG aMemCache, ULONG aPageTotal,
                                  ULONG aAllocVMM, ULONG aFreeVMM,
                                  ULONG aBalloonedVMM, ULONG aSharedVMM,
                                  ULONG aVmNetRx, ULONG aVmNetTx);

    // public methods only for internal purposes

    virtual bool isSessionMachine() const
    {
        return true;
    }

    bool checkForDeath();

    HRESULT onNetworkAdapterChange(INetworkAdapter *networkAdapter, BOOL changeAdapter);
    HRESULT onNATRedirectRuleChange(ULONG ulSlot, BOOL aNatRuleRemove, IN_BSTR aRuleName,
                                 NATProtocol_T aProto, IN_BSTR aHostIp, LONG aHostPort, IN_BSTR aGuestIp, LONG aGuestPort);
    HRESULT onStorageControllerChange();
    HRESULT onMediumChange(IMediumAttachment *aMediumAttachment, BOOL aForce);
    HRESULT onSerialPortChange(ISerialPort *serialPort);
    HRESULT onParallelPortChange(IParallelPort *parallelPort);
    HRESULT onCPUChange(ULONG aCPU, BOOL aRemove);
    HRESULT onCPUExecutionCapChange(ULONG aCpuExecutionCap);
    HRESULT onVRDEServerChange(BOOL aRestart);
    HRESULT onUSBControllerChange();
    HRESULT onUSBDeviceAttach(IUSBDevice *aDevice,
                              IVirtualBoxErrorInfo *aError,
                              ULONG aMaskedIfs);
    HRESULT onUSBDeviceDetach(IN_BSTR aId,
                              IVirtualBoxErrorInfo *aError);
    HRESULT onSharedFolderChange();
    HRESULT onClipboardModeChange(ClipboardMode_T aClipboardMode);
    HRESULT onDragAndDropModeChange(DragAndDropMode_T aDragAndDropMode);
    HRESULT onBandwidthGroupChange(IBandwidthGroup *aBandwidthGroup);
    HRESULT onStorageDeviceChange(IMediumAttachment *aMediumAttachment, BOOL aRemove);

    bool hasMatchingUSBFilter(const ComObjPtr<HostUSBDevice> &aDevice, ULONG *aMaskedIfs);

    HRESULT lockMedia();
    void unlockMedia();

private:

    struct ConsoleTaskData
    {
        ConsoleTaskData()
            : mLastState(MachineState_Null)
        { }

        MachineState_T mLastState;
        ComObjPtr<Progress> mProgress;

        // used when taking snapshot
        ComObjPtr<Snapshot> mSnapshot;

        // used when saving state (either as part of a snapshot or separate)
        Utf8Str strStateFilePath;
    };

    struct Uninit
    {
        enum Reason { Unexpected, Abnormal, Normal };
    };

    struct SnapshotTask;
    struct DeleteSnapshotTask;
    struct RestoreSnapshotTask;

    friend struct DeleteSnapshotTask;
    friend struct RestoreSnapshotTask;

    void uninit(Uninit::Reason aReason);

    HRESULT endSavingState(HRESULT aRC, const Utf8Str &aErrMsg);
    void releaseSavedStateFile(const Utf8Str &strSavedStateFile, Snapshot *pSnapshotToIgnore);

    void deleteSnapshotHandler(DeleteSnapshotTask &aTask);
    void restoreSnapshotHandler(RestoreSnapshotTask &aTask);

    HRESULT prepareDeleteSnapshotMedium(const ComObjPtr<Medium> &aHD,
                                        const Guid &machineId,
                                        const Guid &snapshotId,
                                        bool fOnlineMergePossible,
                                        MediumLockList *aVMMALockList,
                                        ComObjPtr<Medium> &aSource,
                                        ComObjPtr<Medium> &aTarget,
                                        bool &fMergeForward,
                                        ComObjPtr<Medium> &pParentForTarget,
                                        MediaList &aChildrenToReparent,
                                        bool &fNeedOnlineMerge,
                                        MediumLockList * &aMediumLockList);
    void cancelDeleteSnapshotMedium(const ComObjPtr<Medium> &aHD,
                                    const ComObjPtr<Medium> &aSource,
                                    const MediaList &aChildrenToReparent,
                                    bool fNeedsOnlineMerge,
                                    MediumLockList *aMediumLockList,
                                    const Guid &aMediumId,
                                    const Guid &aSnapshotId);
    HRESULT onlineMergeMedium(const ComObjPtr<MediumAttachment> &aMediumAttachment,
                              const ComObjPtr<Medium> &aSource,
                              const ComObjPtr<Medium> &aTarget,
                              bool fMergeForward,
                              const ComObjPtr<Medium> &pParentForTarget,
                              const MediaList &aChildrenToReparent,
                              MediumLockList *aMediumLockList,
                              ComObjPtr<Progress> &aProgress,
                              bool *pfNeedsMachineSaveSettings);

    HRESULT setMachineState(MachineState_T aMachineState);
    HRESULT updateMachineStateOnClient();

    HRESULT mRemoveSavedState;

    ConsoleTaskData mConsoleTaskData;

    /** interprocess semaphore handle for this machine */
#if defined(RT_OS_WINDOWS)
    HANDLE mIPCSem;
    Bstr mIPCSemName;
    friend bool Machine::isSessionOpen(ComObjPtr<SessionMachine> &aMachine,
                                       ComPtr<IInternalSessionControl> *aControl,
                                       HANDLE *aIPCSem, bool aAllowClosing);
#elif defined(RT_OS_OS2)
    HMTX mIPCSem;
    Bstr mIPCSemName;
    friend bool Machine::isSessionOpen(ComObjPtr<SessionMachine> &aMachine,
                                       ComPtr<IInternalSessionControl> *aControl,
                                       HMTX *aIPCSem, bool aAllowClosing);
#elif defined(VBOX_WITH_SYS_V_IPC_SESSION_WATCHER)
    int mIPCSem;
# ifdef VBOX_WITH_NEW_SYS_V_KEYGEN
    Bstr mIPCKey;
# endif /*VBOX_WITH_NEW_SYS_V_KEYGEN */
#else
# error "Port me!"
#endif

    static DECLCALLBACK(int) taskHandler(RTTHREAD thread, void *pvUser);
};

// SnapshotMachine class
////////////////////////////////////////////////////////////////////////////////

/**
 *  @note Notes on locking objects of this class:
 *  SnapshotMachine shares some data with the primary Machine instance (pointed
 *  to by the |mPeer| member). In order to provide data consistency it also
 *  shares its lock handle. This means that whenever you lock a SessionMachine
 *  instance using Auto[Reader]Lock or AutoMultiLock, the corresponding Machine
 *  instance is also locked in the same lock mode. Keep it in mind.
 */
class ATL_NO_VTABLE SnapshotMachine :
    public Machine
{
public:
    VIRTUALBOXBASE_ADD_ERRORINFO_SUPPORT(SnapshotMachine, IMachine)

    DECLARE_NOT_AGGREGATABLE(SnapshotMachine)

    DECLARE_PROTECT_FINAL_CONSTRUCT()

    BEGIN_COM_MAP(SnapshotMachine)
        VBOX_DEFAULT_INTERFACE_ENTRIES(IMachine)
    END_COM_MAP()

    DECLARE_EMPTY_CTOR_DTOR(SnapshotMachine)

    HRESULT FinalConstruct();
    void FinalRelease();

    // public initializer/uninitializer for internal purposes only
    HRESULT init(SessionMachine *aSessionMachine,
                 IN_GUID aSnapshotId,
                 const Utf8Str &aStateFilePath);
    HRESULT initFromSettings(Machine *aMachine,
                             const settings::Hardware &hardware,
                             const settings::Debugging *pDbg,
                             const settings::Autostart *pAutostart,
                             const settings::Storage &storage,
                             IN_GUID aSnapshotId,
                             const Utf8Str &aStateFilePath);
    void uninit();

    // util::Lockable interface
    RWLockHandle *lockHandle() const;

    // public methods only for internal purposes

    virtual bool isSnapshotMachine() const
    {
        return true;
    }

    HRESULT onSnapshotChange(Snapshot *aSnapshot);

    // unsafe inline public methods for internal purposes only (ensure there is
    // a caller and a read lock before calling them!)

    const Guid& getSnapshotId() const { return mSnapshotId; }

private:

    Guid mSnapshotId;
    /** This field replaces mPeer for SessionMachine instances, as having
     * a peer reference is plain meaningless and causes many subtle problems
     * with saving settings and the like. */
    Machine * const mMachine;

    friend class Snapshot;
};

// third party methods that depend on SnapshotMachine definition

inline const Guid &Machine::getSnapshotId() const
{
    return (isSnapshotMachine())
                ? static_cast<const SnapshotMachine*>(this)->getSnapshotId()
                : Guid::Empty;
}


#endif // ____H_MACHINEIMPL
/* vi: set tabstop=4 shiftwidth=4 expandtab: */
