/* $Id: PerformanceImpl.h $ */

/** @file
 *
 * VBox Performance COM class implementation.
 */

/*
 * Copyright (C) 2008-2012 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#ifndef ____H_PERFORMANCEIMPL
#define ____H_PERFORMANCEIMPL

#include "VirtualBoxBase.h"

#include <VBox/com/com.h>
#include <VBox/com/array.h>
//#ifdef VBOX_WITH_RESOURCE_USAGE_API
#include <iprt/timer.h>
//#endif /* VBOX_WITH_RESOURCE_USAGE_API */

#include <list>

namespace pm
{
    class Metric;
    class BaseMetric;
    class CollectorHAL;
    class CollectorGuest;
    class CollectorGuestManager;
}

#undef min
#undef max

/* Each second we obtain new CPU load stats. */
#define VBOX_USAGE_SAMPLER_MIN_INTERVAL 1000

class HostUSBDevice;

class ATL_NO_VTABLE PerformanceMetric :
    public VirtualBoxBase,
    VBOX_SCRIPTABLE_IMPL(IPerformanceMetric)
{
public:
    VIRTUALBOXBASE_ADD_ERRORINFO_SUPPORT(PerformanceMetric, IPerformanceMetric)

    DECLARE_NOT_AGGREGATABLE (PerformanceMetric)

    DECLARE_PROTECT_FINAL_CONSTRUCT()

    BEGIN_COM_MAP (PerformanceMetric)
        VBOX_DEFAULT_INTERFACE_ENTRIES (IPerformanceMetric)
    END_COM_MAP()

    DECLARE_EMPTY_CTOR_DTOR (PerformanceMetric)

    HRESULT FinalConstruct();
    void FinalRelease();

    // public initializer/uninitializer for internal purposes only
    HRESULT init (pm::Metric *aMetric);
    HRESULT init (pm::BaseMetric *aMetric);
    void uninit();

    // IPerformanceMetric properties
    STDMETHOD(COMGETTER(MetricName)) (BSTR *aMetricName);
    STDMETHOD(COMGETTER(Object)) (IUnknown **anObject);
    STDMETHOD(COMGETTER(Description)) (BSTR *aDescription);
    STDMETHOD(COMGETTER(Period)) (ULONG *aPeriod);
    STDMETHOD(COMGETTER(Count)) (ULONG *aCount);
    STDMETHOD(COMGETTER(Unit)) (BSTR *aUnit);
    STDMETHOD(COMGETTER(MinimumValue)) (LONG *aMinValue);
    STDMETHOD(COMGETTER(MaximumValue)) (LONG *aMaxValue);

    // IPerformanceMetric methods

    // public methods only for internal purposes

    // public methods for internal purposes only
    // (ensure there is a caller and a read lock before calling them!)

private:

    struct Data
    {
        /* Constructor. */
        Data()
            : period(0), count(0), min(0), max(0)
        {
        }

        Bstr             name;
        ComPtr<IUnknown> object;
        Bstr             description;
        ULONG            period;
        ULONG            count;
        Bstr             unit;
        LONG             min;
        LONG             max;
    };

    Data m;
};


class ATL_NO_VTABLE PerformanceCollector :
    public VirtualBoxBase,
    VBOX_SCRIPTABLE_IMPL(IPerformanceCollector)
{
public:

    VIRTUALBOXBASE_ADD_ERRORINFO_SUPPORT(PerformanceCollector, IPerformanceCollector)

    DECLARE_NOT_AGGREGATABLE (PerformanceCollector)

    DECLARE_PROTECT_FINAL_CONSTRUCT()

    BEGIN_COM_MAP(PerformanceCollector)
        VBOX_DEFAULT_INTERFACE_ENTRIES(IPerformanceCollector)
    END_COM_MAP()

    DECLARE_EMPTY_CTOR_DTOR (PerformanceCollector)

    HRESULT FinalConstruct();
    void FinalRelease();

    // public initializers/uninitializers only for internal purposes
    HRESULT init();
    void uninit();

    // IPerformanceCollector properties
    STDMETHOD(COMGETTER(MetricNames)) (ComSafeArrayOut (BSTR, metricNames));

    // IPerformanceCollector methods
    STDMETHOD(GetMetrics) (ComSafeArrayIn (IN_BSTR, metricNames),
                           ComSafeArrayIn (IUnknown *, objects),
                           ComSafeArrayOut (IPerformanceMetric *, outMetrics));
    STDMETHOD(SetupMetrics) (ComSafeArrayIn (IN_BSTR, metricNames),
                             ComSafeArrayIn (IUnknown *, objects),
                             ULONG aPeriod, ULONG aCount,
                             ComSafeArrayOut (IPerformanceMetric *,
                                              outMetrics));
    STDMETHOD(EnableMetrics) (ComSafeArrayIn (IN_BSTR, metricNames),
                              ComSafeArrayIn (IUnknown *, objects),
                              ComSafeArrayOut (IPerformanceMetric *,
                                               outMetrics));
    STDMETHOD(DisableMetrics) (ComSafeArrayIn (IN_BSTR, metricNames),
                               ComSafeArrayIn (IUnknown *, objects),
                               ComSafeArrayOut (IPerformanceMetric *,
                                                outMetrics));
    STDMETHOD(QueryMetricsData) (ComSafeArrayIn (IN_BSTR, metricNames),
                                 ComSafeArrayIn (IUnknown *, objects),
                                 ComSafeArrayOut (BSTR, outMetricNames),
                                 ComSafeArrayOut (IUnknown *, outObjects),
                                 ComSafeArrayOut (BSTR, outUnits),
                                 ComSafeArrayOut (ULONG, outScales),
                                 ComSafeArrayOut (ULONG, outSequenceNumbers),
                                 ComSafeArrayOut (ULONG, outDataIndices),
                                 ComSafeArrayOut (ULONG, outDataLengths),
                                 ComSafeArrayOut (LONG, outData));

    // public methods only for internal purposes

    void registerBaseMetric (pm::BaseMetric *baseMetric);
    void registerMetric (pm::Metric *metric);
    void unregisterBaseMetricsFor (const ComPtr<IUnknown> &object, const Utf8Str name = "*");
    void unregisterMetricsFor (const ComPtr<IUnknown> &object, const Utf8Str name = "*");
    void registerGuest(pm::CollectorGuest* pGuest);
    void unregisterGuest(pm::CollectorGuest* pGuest);

    void suspendSampling();
    void resumeSampling();

    // public methods for internal purposes only
    // (ensure there is a caller and a read lock before calling them!)

    pm::CollectorHAL          *getHAL()          { return m.hal; };
    pm::CollectorGuestManager *getGuestManager() { return m.gm; };

private:
    HRESULT toIPerformanceMetric(pm::Metric *src, IPerformanceMetric **dst);
    HRESULT toIPerformanceMetric(pm::BaseMetric *src, IPerformanceMetric **dst);

    static void staticSamplerCallback (RTTIMERLR hTimerLR, void *pvUser, uint64_t iTick);
    void samplerCallback(uint64_t iTick);

    const Utf8Str& getFailedGuestName();

    typedef std::list<pm::Metric*> MetricList;
    typedef std::list<pm::BaseMetric*> BaseMetricList;

    enum
    {
        MAGIC = 0xABBA1972u
    };

    unsigned int mMagic;
    const Utf8Str mUnknownGuest;

    struct Data
    {
        Data() : hal(0) {};

        BaseMetricList             baseMetrics;
        MetricList                 metrics;
        RTTIMERLR                  sampler;
        pm::CollectorHAL          *hal;
        pm::CollectorGuestManager *gm;
    };

    Data m;
};

#endif //!____H_PERFORMANCEIMPL
/* vi: set tabstop=4 shiftwidth=4 expandtab: */
