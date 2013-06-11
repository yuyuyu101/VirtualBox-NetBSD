<?xml version="1.0"?>
<!-- $Id: midl.xsl $ -->

<!--
 *  A template to generate a MS IDL compatible interface definition file
 *  from the generic interface definition expressed in XML.

    Copyright (C) 2006-2012 Oracle Corporation

    This file is part of VirtualBox Open Source Edition (OSE), as
    available from http://www.virtualbox.org. This file is free software;
    you can redistribute it and/or modify it under the terms of the GNU
    General Public License (GPL) as published by the Free Software
    Foundation, in version 2 as it comes in the "COPYING" file of the
    VirtualBox OSE distribution. VirtualBox OSE is distributed in the
    hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
-->

<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<xsl:strip-space elements="*"/>


<!--
//  helper definitions
/////////////////////////////////////////////////////////////////////////////
-->

<!--
 *  capitalizes the first letter
-->
<xsl:template name="capitalize">
  <xsl:param name="str" select="."/>
  <xsl:value-of select="
    concat(
      translate(substring($str,1,1),'abcdefghijklmnopqrstuvwxyz','ABCDEFGHIJKLMNOPQRSTUVWXYZ'),
      substring($str,2)
    )
  "/>
</xsl:template>

<!--
 *  uncapitalizes the first letter only if the second one is not capital
 *  otherwise leaves the string unchanged
-->
<xsl:template name="uncapitalize">
  <xsl:param name="str" select="."/>
  <xsl:choose>
    <xsl:when test="not(contains('ABCDEFGHIJKLMNOPQRSTUVWXYZ', substring($str,2,1)))">
      <xsl:value-of select="
        concat(
          translate(substring($str,1,1),'ABCDEFGHIJKLMNOPQRSTUVWXYZ','abcdefghijklmnopqrstuvwxyz'),
          substring($str,2)
        )
      "/>
    </xsl:when>
    <xsl:otherwise>
      <xsl:value-of select="string($str)"/>
    </xsl:otherwise>
  </xsl:choose>
</xsl:template>


<!--
//  templates
/////////////////////////////////////////////////////////////////////////////
-->

<!--
 *  not explicitly matched elements and attributes
-->
<xsl:template match="*"/>


<!--
 *  header
-->
<xsl:template match="/idl">
  <xsl:text>
/*
 *  DO NOT EDIT! This is a generated file.
 *
 *  MS IDL (MIDL) definition for VirtualBox Main API (COM interfaces)
 *  generated from XIDL (XML interface definition).
 *
 *  Source    : src/VBox/Main/idl/VirtualBox.xidl
 *  Generator : src/VBox/Main/idl/midl.xsl
 */
  </xsl:text>
  <xsl:text>&#x0A;</xsl:text>
  <xsl:text>import "unknwn.idl";&#x0A;&#x0A;</xsl:text>
  <xsl:apply-templates/>
</xsl:template>


<!--
 *  ignore all |if|s except those for MIDL target
-->
<xsl:template match="if">
  <xsl:if test="@target='midl'">
    <xsl:apply-templates/>
  </xsl:if>
</xsl:template>
<xsl:template match="if" mode="forward">
  <xsl:if test="@target='midl'">
    <xsl:apply-templates mode="forward"/>
  </xsl:if>
</xsl:template>
<xsl:template match="if" mode="forwarder">
  <xsl:param name="nameOnly"/>
  <xsl:if test="@target='midl'">
    <xsl:apply-templates mode="forwarder">
      <xsl:with-param name="nameOnly" select="$nameOnly"/>
    </xsl:apply-templates>
  </xsl:if>
</xsl:template>


<!--
 *  cpp_quote
-->
<xsl:template match="cpp">
  <xsl:text>cpp_quote("</xsl:text>
  <xsl:value-of select="@line"/>
  <xsl:text>")&#x0A;&#x0A;</xsl:text>
</xsl:template>


<!--
 *  #if statement (@if attribute)
-->
<xsl:template match="@if" mode="begin">
  <xsl:text>#if </xsl:text>
  <xsl:value-of select="."/>
  <xsl:text>&#x0A;</xsl:text>
</xsl:template>
<xsl:template match="@if" mode="end">
  <xsl:text>#endif&#x0A;</xsl:text>
</xsl:template>


<!--
 *  libraries
-->
<xsl:template match="library">[
    uuid(<xsl:value-of select="@uuid"/>),
    version(<xsl:value-of select="@version"/>),
    helpstring("<xsl:value-of select="@desc"/>")
]
<xsl:text>library </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text>&#x0A;{&#x0A;</xsl:text>
  <xsl:text>&#x0A;importlib("stdole2.tlb");&#x0A;&#x0A;</xsl:text>
  <!-- result codes -->
  <xsl:for-each select="result">
    <xsl:apply-templates select="."/>
  </xsl:for-each>
  <xsl:text>&#x0A;</xsl:text>
  <!-- forward declarations -->
  <xsl:apply-templates select="if | interface" mode="forward"/>
  <xsl:text>&#x0A;</xsl:text>
  <!-- all enums go first -->
  <xsl:apply-templates select="enum | if/enum"/>
  <!-- everything else but result codes and enums -->
  <xsl:apply-templates select="*[not(self::result or self::enum) and
                                 not(self::if[result] or self::if[enum])]"/>
  <!-- -->
  <xsl:text>}; /* library </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text> */&#x0A;&#x0A;</xsl:text>
</xsl:template>


<!--
 *  result codes
-->
<xsl:template match="result">
  <xsl:text>cpp_quote("</xsl:text>
  <xsl:value-of select="concat('#define ',@name,' ',@value)"/>
  <xsl:text>")&#x0A;</xsl:text>
</xsl:template>


<!--
 *  forward declarations
-->
<xsl:template match="interface" mode="forward">
  <xsl:text>interface </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text>;&#x0A;</xsl:text>
</xsl:template>


<!--
 *  interfaces
-->
<xsl:template match="interface">[
    uuid(<xsl:value-of select="@uuid"/>),
    object,
    dual,
    oleautomation
]
<xsl:text>interface </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text> : </xsl:text>
  <xsl:choose>
    <xsl:when test="@extends='$unknown'">IDispatch</xsl:when>
    <xsl:when test="@extends='$dispatched'">IDispatch</xsl:when>
    <xsl:when test="@extends='$errorinfo'">IErrorInfo</xsl:when>
    <xsl:otherwise><xsl:value-of select="@extends"/></xsl:otherwise>
  </xsl:choose>
  <xsl:text>&#x0A;{&#x0A;</xsl:text>
  <!-- attributes (properties) -->
  <xsl:apply-templates select="attribute"/>
  <!-- methods -->
  <xsl:apply-templates select="method"/>
  <!-- 'if' enclosed elements, unsorted -->
  <xsl:apply-templates select="if"/>
  <!-- -->
  <xsl:text>}; /* interface </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text> */&#x0A;&#x0A;</xsl:text>
  <!-- Interface implementation forwarder macro -->
  <xsl:text>/* Interface implementation forwarder macro */&#x0A;</xsl:text>
  <!-- 1) individual methods -->
  <xsl:apply-templates select="attribute" mode="forwarder"/>
  <xsl:apply-templates select="method" mode="forwarder"/>
  <xsl:apply-templates select="if" mode="forwarder"/>
  <!-- 2) COM_FORWARD_Interface_TO(smth) -->
  <xsl:text>cpp_quote("#define COM_FORWARD_</xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text>_TO(smth) </xsl:text>
  <xsl:apply-templates select="attribute" mode="forwarder">
    <xsl:with-param name="nameOnly" select="'yes'"/>
  </xsl:apply-templates>
  <xsl:apply-templates select="method" mode="forwarder">
    <xsl:with-param name="nameOnly" select="'yes'"/>
  </xsl:apply-templates>
  <xsl:apply-templates select="if" mode="forwarder">
    <xsl:with-param name="nameOnly" select="'yes'"/>
  </xsl:apply-templates>
  <xsl:text>")&#x0A;</xsl:text>
  <!-- 3) COM_FORWARD_Interface_TO_OBJ(obj) -->
  <xsl:text>cpp_quote("#define COM_FORWARD_</xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text>_TO_OBJ(obj) COM_FORWARD_</xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text>_TO ((obj)->)")&#x0A;</xsl:text>
  <!-- 4) COM_FORWARD_Interface_TO_BASE(base) -->
  <xsl:text>cpp_quote("#define COM_FORWARD_</xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text>_TO_BASE(base) COM_FORWARD_</xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text>_TO (base::)")&#x0A;</xsl:text>
  <!-- end -->
  <xsl:text>&#x0A;</xsl:text>
</xsl:template>


<!--
 *  attributes
-->
<xsl:template match="interface//attribute">
  <xsl:apply-templates select="@if" mode="begin"/>
  <!-- getter -->
  <xsl:text>    [propget] HRESULT </xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text> ([out, retval] </xsl:text>
  <xsl:if test="@safearray='yes'">
    <xsl:text>SAFEARRAY(</xsl:text>
  </xsl:if>
  <xsl:apply-templates select="@type"/>
  <xsl:if test="@safearray='yes'">
    <xsl:text>)</xsl:text>
  </xsl:if>
  <xsl:text> * a</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text>);&#x0A;</xsl:text>
  <!-- setter -->
  <xsl:if test="not(@readonly='yes')">
    <xsl:text>    [propput] HRESULT </xsl:text>
    <xsl:call-template name="capitalize">
      <xsl:with-param name="str" select="@name"/>
    </xsl:call-template>
    <xsl:text> ([in] </xsl:text>
    <xsl:if test="@safearray='yes'">
      <xsl:text>SAFEARRAY(</xsl:text>
    </xsl:if>
    <xsl:apply-templates select="@type"/>
    <xsl:if test="@safearray='yes'">
      <xsl:text>)</xsl:text>
    </xsl:if>
    <xsl:text> a</xsl:text>
    <xsl:call-template name="capitalize">
      <xsl:with-param name="str" select="@name"/>
    </xsl:call-template>
    <xsl:text>);&#x0A;</xsl:text>
  </xsl:if>
  <xsl:apply-templates select="@if" mode="end"/>
  <xsl:text>&#x0A;</xsl:text>
</xsl:template>

<xsl:template match="interface//attribute" mode="forwarder">

  <!-- if nameOnly='yes' then only the macro name is composed
       followed by a space -->
  <xsl:param name="nameOnly"/>

  <xsl:variable name="parent" select="ancestor::interface"/>

  <xsl:apply-templates select="@if" mode="begin"/>

  <xsl:choose>
    <xsl:when test="$nameOnly='yes'">
      <!-- getter: COM_FORWARD_Interface_GETTER_Name_TO(smth) -->
      <xsl:text>COM_FORWARD_</xsl:text>
      <xsl:value-of select="$parent/@name"/>
      <xsl:text>_GETTER_</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text>_TO (smth) </xsl:text>
      <!-- setter: COM_FORWARD_Interface_SETTER_Name_TO(smth) -->
      <xsl:if test="not(@readonly='yes')">
        <xsl:text>COM_FORWARD_</xsl:text>
        <xsl:value-of select="$parent/@name"/>
        <xsl:text>_SETTER_</xsl:text>
        <xsl:call-template name="capitalize">
          <xsl:with-param name="str" select="@name"/>
        </xsl:call-template>
        <xsl:text>_TO (smth) </xsl:text>
      </xsl:if>
    </xsl:when>
    <xsl:otherwise>
      <!-- getter: COM_FORWARD_Interface_GETTER_Name_TO(smth) -->
      <xsl:text>cpp_quote("#define COM_FORWARD_</xsl:text>
      <xsl:value-of select="$parent/@name"/>
      <xsl:text>_GETTER_</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text>_TO(smth) HRESULT STDMETHODCALLTYPE get_</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text> (</xsl:text>
      <xsl:choose>
        <xsl:when test="@safearray='yes'">
          <xsl:text>SAFEARRAY *</xsl:text>
        </xsl:when>
        <xsl:otherwise>
          <xsl:apply-templates select="@type"/>
        </xsl:otherwise>
      </xsl:choose>
      <xsl:text> * a</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text>) { return smth get_</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text> (a</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text>); }")&#x0A;</xsl:text>
      <!-- getter: COM_FORWARD_Interface_GETTER_Name_TO_OBJ(obj) -->
      <xsl:text>cpp_quote("#define COM_FORWARD_</xsl:text>
      <xsl:value-of select="$parent/@name"/>
      <xsl:text>_GETTER_</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text>_TO_OBJ(obj) COM_FORWARD_</xsl:text>
      <xsl:value-of select="$parent/@name"/>
      <xsl:text>_GETTER_</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text>_TO ((obj)->)")&#x0A;</xsl:text>
      <!-- getter: COM_FORWARD_Interface_GETTER_Name_TO_BASE(base) -->
      <xsl:text>cpp_quote("#define COM_FORWARD_</xsl:text>
      <xsl:value-of select="$parent/@name"/>
      <xsl:text>_GETTER_</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text>_TO_BASE(base) COM_FORWARD_</xsl:text>
      <xsl:value-of select="$parent/@name"/>
      <xsl:text>_GETTER_</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text>_TO (base::)")&#x0A;</xsl:text>
      <!-- -->
      <xsl:if test="not(@readonly='yes')">
        <!-- setter: COM_FORWARD_Interface_SETTER_Name_TO(smth) -->
        <xsl:text>cpp_quote("#define COM_FORWARD_</xsl:text>
        <xsl:value-of select="$parent/@name"/>
        <xsl:text>_SETTER_</xsl:text>
        <xsl:call-template name="capitalize">
          <xsl:with-param name="str" select="@name"/>
        </xsl:call-template>
        <xsl:text>_TO(smth) HRESULT STDMETHODCALLTYPE put_</xsl:text>
        <xsl:call-template name="capitalize">
          <xsl:with-param name="str" select="@name"/>
        </xsl:call-template>
        <xsl:text> (</xsl:text>
        <xsl:choose>
          <xsl:when test="@safearray='yes'">
            <xsl:text>SAFEARRAY *</xsl:text>
          </xsl:when>
          <xsl:otherwise>
            <xsl:apply-templates select="@type"/>
          </xsl:otherwise>
        </xsl:choose>
        <xsl:text> a</xsl:text>
        <xsl:call-template name="capitalize">
          <xsl:with-param name="str" select="@name"/>
        </xsl:call-template>
        <xsl:text>) { return smth put_</xsl:text>
        <xsl:call-template name="capitalize">
          <xsl:with-param name="str" select="@name"/>
        </xsl:call-template>
        <xsl:text> (a</xsl:text>
        <xsl:call-template name="capitalize">
          <xsl:with-param name="str" select="@name"/>
        </xsl:call-template>
        <xsl:text>); }")&#x0A;</xsl:text>
        <!-- setter: COM_FORWARD_Interface_SETTER_Name_TO_OBJ(obj) -->
        <xsl:text>cpp_quote("#define COM_FORWARD_</xsl:text>
        <xsl:value-of select="$parent/@name"/>
        <xsl:text>_SETTER_</xsl:text>
        <xsl:call-template name="capitalize">
          <xsl:with-param name="str" select="@name"/>
        </xsl:call-template>
        <xsl:text>_TO_OBJ(obj) COM_FORWARD_</xsl:text>
        <xsl:value-of select="$parent/@name"/>
        <xsl:text>_SETTER_</xsl:text>
        <xsl:call-template name="capitalize">
          <xsl:with-param name="str" select="@name"/>
        </xsl:call-template>
        <xsl:text>_TO ((obj)->)")&#x0A;</xsl:text>
        <!-- setter: COM_FORWARD_Interface_SETTER_Name_TO_BASE(base) -->
        <xsl:text>cpp_quote("#define COM_FORWARD_</xsl:text>
        <xsl:value-of select="$parent/@name"/>
        <xsl:text>_SETTER_</xsl:text>
        <xsl:call-template name="capitalize">
          <xsl:with-param name="str" select="@name"/>
        </xsl:call-template>
        <xsl:text>_TO_BASE(base) COM_FORWARD_</xsl:text>
        <xsl:value-of select="$parent/@name"/>
        <xsl:text>_SETTER_</xsl:text>
        <xsl:call-template name="capitalize">
          <xsl:with-param name="str" select="@name"/>
        </xsl:call-template>
        <xsl:text>_TO (base::)")&#x0A;</xsl:text>
      </xsl:if>
    </xsl:otherwise>
  </xsl:choose>

  <xsl:apply-templates select="@if" mode="end"/>

</xsl:template>


<!--
 *  methods
-->
<xsl:template match="interface//method">
  <xsl:apply-templates select="@if" mode="begin"/>
  <xsl:text>    HRESULT </xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:choose>
    <xsl:when test="param">
      <xsl:text> (&#x0A;</xsl:text>
      <xsl:for-each select="param [position() != last()]">
        <xsl:text>        </xsl:text>
        <xsl:apply-templates select="."/>
        <xsl:text>,&#x0A;</xsl:text>
      </xsl:for-each>
      <xsl:text>        </xsl:text>
      <xsl:apply-templates select="param [last()]"/>
      <xsl:text>&#x0A;    );&#x0A;</xsl:text>
    </xsl:when>
    <xsl:otherwise test="not(param)">
      <xsl:text>();&#x0A;</xsl:text>
    </xsl:otherwise>
  </xsl:choose>
  <xsl:apply-templates select="@if" mode="end"/>
  <xsl:text>&#x0A;</xsl:text>
</xsl:template>

<xsl:template match="interface//method" mode="forwarder">

  <!-- if nameOnly='yes' then only the macro name is composed followed by \ -->
  <xsl:param name="nameOnly"/>

  <xsl:variable name="parent" select="ancestor::interface"/>

  <xsl:apply-templates select="@if" mode="begin"/>

  <xsl:choose>
    <xsl:when test="$nameOnly='yes'">
      <!-- COM_FORWARD_Interface_Method_TO(smth) -->
      <xsl:text>COM_FORWARD_</xsl:text>
      <xsl:value-of select="$parent/@name"/>
      <xsl:text>_</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text>_TO (smth) </xsl:text>
    </xsl:when>
    <xsl:otherwise>
      <xsl:text>cpp_quote("#define COM_FORWARD_</xsl:text>
      <xsl:value-of select="$parent/@name"/>
      <xsl:text>_</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text>_TO(smth) HRESULT STDMETHODCALLTYPE </xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:choose>
        <xsl:when test="param">
          <xsl:text> (</xsl:text>
          <xsl:for-each select="param [position() != last()]">
            <xsl:apply-templates select="." mode="forwarder"/>
            <xsl:text>, </xsl:text>
          </xsl:for-each>
          <xsl:apply-templates select="param [last()]" mode="forwarder"/>
          <xsl:text>) { return smth </xsl:text>
          <xsl:call-template name="capitalize">
            <xsl:with-param name="str" select="@name"/>
          </xsl:call-template>
          <xsl:text> (</xsl:text>
          <xsl:for-each select="param [position() != last()]">
            <xsl:text>a</xsl:text>
            <xsl:call-template name="capitalize">
              <xsl:with-param name="str" select="@name"/>
            </xsl:call-template>
            <xsl:text>, </xsl:text>
          </xsl:for-each>
          <xsl:text>a</xsl:text>
          <xsl:call-template name="capitalize">
            <xsl:with-param name="str" select="param [last()]/@name"/>
          </xsl:call-template>
          <xsl:text>); }</xsl:text>
        </xsl:when>
        <xsl:otherwise test="not(param)">
          <xsl:text>() { return smth </xsl:text>
          <xsl:call-template name="capitalize">
            <xsl:with-param name="str" select="@name"/>
          </xsl:call-template>
          <xsl:text>(); }</xsl:text>
        </xsl:otherwise>
      </xsl:choose>
      <xsl:text>")&#x0A;</xsl:text>
      <!-- COM_FORWARD_Interface_Method_TO_OBJ(obj) -->
      <xsl:text>cpp_quote("#define COM_FORWARD_</xsl:text>
      <xsl:value-of select="$parent/@name"/>
      <xsl:text>_</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text>_TO_OBJ(obj) COM_FORWARD_</xsl:text>
      <xsl:value-of select="$parent/@name"/>
      <xsl:text>_</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text>_TO ((obj)->)")&#x0A;</xsl:text>
      <!-- COM_FORWARD_Interface_Method_TO_BASE(base) -->
      <xsl:text>cpp_quote("#define COM_FORWARD_</xsl:text>
      <xsl:value-of select="$parent/@name"/>
      <xsl:text>_</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text>_TO_BASE(base) COM_FORWARD_</xsl:text>
      <xsl:value-of select="$parent/@name"/>
      <xsl:text>_</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text>_TO (base::)")&#x0A;</xsl:text>
    </xsl:otherwise>
  </xsl:choose>

  <xsl:apply-templates select="@if" mode="end"/>

</xsl:template>


<!--
 *  modules
-->
<xsl:template match="module">
  <xsl:apply-templates select="class"/>
</xsl:template>


<!--
 *  co-classes
-->
<xsl:template match="module/class">[
    uuid(<xsl:value-of select="@uuid"/>)
]
<xsl:text>coclass </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text>&#x0A;{&#x0A;</xsl:text>
  <xsl:for-each select="interface">
    <xsl:text>    </xsl:text>
    <xsl:if test="@default='yes'">
      <xsl:text>[default] </xsl:text>
    </xsl:if>
    <xsl:text>interface </xsl:text>
    <xsl:value-of select="@name"/>
    <xsl:text>;&#x0A;</xsl:text>
  </xsl:for-each>
  <xsl:for-each select="eventsink">
    <xsl:text>    </xsl:text>
    <xsl:choose>
      <xsl:when test="@default='yes'"><xsl:text>[default,source]</xsl:text></xsl:when>
      <xsl:otherwise><xsl:text>[source]</xsl:text></xsl:otherwise>
    </xsl:choose>
    <xsl:text> interface </xsl:text>
    <xsl:value-of select="@name"/>
    <xsl:text>;&#x0A;</xsl:text>
  </xsl:for-each>
  <xsl:text>&#x0A;}; /* coclass </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text> */&#x0A;&#x0A;</xsl:text>
</xsl:template>


<!--
 *  enums
-->
<xsl:template match="enum">[
    uuid(<xsl:value-of select="@uuid"/>),
    v1_enum
]
<xsl:text>typedef enum &#x0A;{&#x0A;</xsl:text>
  <xsl:for-each select="const">
    <xsl:text>    </xsl:text>
    <xsl:value-of select="concat(../@name,'_',@name)"/> = <xsl:value-of select="@value"/>
    <xsl:choose>
      <xsl:when test="position()!=last()"><xsl:text>,&#x0A;</xsl:text></xsl:when>
      <xsl:otherwise><xsl:text>&#x0A;</xsl:text></xsl:otherwise>
    </xsl:choose>
  </xsl:for-each>
  <xsl:text>} </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text>;&#x0A;&#x0A;</xsl:text>
  <!-- -->
  <xsl:value-of select="concat('/* cross-platform type name for ', @name, ' */&#x0A;')"/>
  <xsl:value-of select="concat('cpp_quote(&quot;#define ', @name, '_T', ' ',
                               @name, '&quot;)&#x0A;&#x0A;')"/>
  <xsl:text>&#x0A;&#x0A;</xsl:text>
</xsl:template>


<!--
 *  method parameters
-->
<xsl:template match="method/param">
  <xsl:text>[</xsl:text>
  <xsl:choose>
    <xsl:when test="@dir='in'">in</xsl:when>
    <xsl:when test="@dir='out'">out</xsl:when>
    <xsl:when test="@dir='return'">out, retval</xsl:when>
    <xsl:otherwise>in</xsl:otherwise>
  </xsl:choose>
  <xsl:text>] </xsl:text>
  <xsl:if test="@safearray='yes'">
    <xsl:text>SAFEARRAY(</xsl:text>
  </xsl:if>
  <xsl:apply-templates select="@type"/>
  <xsl:if test="@safearray='yes'">
    <xsl:text>)</xsl:text>
  </xsl:if>
  <xsl:if test="@dir='out' or @dir='return'">
    <xsl:text> *</xsl:text>
  </xsl:if>
  <xsl:text> a</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
</xsl:template>

<xsl:template match="method/param" mode="forwarder">
  <xsl:choose>
    <xsl:when test="@safearray='yes'">
      <xsl:text>SAFEARRAY *</xsl:text>
    </xsl:when>
    <xsl:otherwise>
      <xsl:apply-templates select="@type"/>
    </xsl:otherwise>
  </xsl:choose>
  <xsl:if test="@dir='out' or @dir='return' or @safearray='yes'">
    <xsl:text> *</xsl:text>
  </xsl:if>
  <xsl:text> a</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
</xsl:template>


<!--
 *  attribute/parameter type conversion
-->
<xsl:template match="attribute/@type | param/@type">
  <xsl:variable name="self_target" select="current()/ancestor::if/@target"/>

  <xsl:choose>
    <!-- modifiers -->
    <xsl:when test="name(current())='type' and ../@mod">
      <xsl:choose>
        <xsl:when test="../@mod='ptr'">
          <xsl:choose>
            <!-- standard types -->
            <!--xsl:when test=".='result'">??</xsl:when-->
            <xsl:when test=".='boolean'">BOOL *</xsl:when>
            <xsl:when test=".='octet'">BYTE *</xsl:when>
            <xsl:when test=".='short'">SHORT *</xsl:when>
            <xsl:when test=".='unsigned short'">USHORT *</xsl:when>
            <xsl:when test=".='long'">LONG *</xsl:when>
            <xsl:when test=".='long long'">LONG64 *</xsl:when>
            <xsl:when test=".='unsigned long'">ULONG *</xsl:when>
            <xsl:when test=".='unsigned long long'">
              <xsl:message terminate="yes">
                 <xsl:value-of select="'&quot;unsigned long long&quot; no longer supported'" />
              </xsl:message>
            </xsl:when>
            <xsl:otherwise>
              <xsl:message terminate="yes">
                <xsl:value-of select="concat(../../../@name,'::',../../@name,'::',../@name,': ')"/>
                <xsl:text>attribute 'mod=</xsl:text>
                <xsl:value-of select="concat('&quot;',../@mod,'&quot;')"/>
                <xsl:text>' cannot be used with type </xsl:text>
                <xsl:value-of select="concat('&quot;',current(),'&quot;!')"/>
              </xsl:message>
            </xsl:otherwise>
          </xsl:choose>
        </xsl:when>
        <xsl:when test="../@mod='string'">
          <xsl:choose>
            <!-- standard types -->
            <!--xsl:when test=".='result'">??</xsl:when-->
            <xsl:when test=".='uuid'">BSTR</xsl:when>
            <xsl:otherwise>
              <xsl:message terminate="yes">
                <xsl:value-of select="concat(../../../@name,'::',../../@name,'::',../@name,': ')"/>
                <xsl:text>attribute 'mod=</xsl:text>
                <xsl:value-of select="concat('&quot;',../@mod,'&quot;')"/>
                <xsl:text>' cannot be used with type </xsl:text>
                <xsl:value-of select="concat('&quot;',current(),'&quot;!')"/>
              </xsl:message>
            </xsl:otherwise>
          </xsl:choose>
        </xsl:when>
        <xsl:otherwise>
          <xsl:message terminate="yes">
            <xsl:value-of select="concat(../../../@name,'::',../../@name,'::',../@name,': ')"/>
            <xsl:value-of select="concat('value &quot;',../@mod,'&quot; ')"/>
            <xsl:text>of attribute 'mod' is invalid!</xsl:text>
          </xsl:message>
        </xsl:otherwise>
      </xsl:choose>
    </xsl:when>
    <!-- no modifiers -->
    <xsl:otherwise>
      <xsl:choose>
        <!-- standard types -->
        <xsl:when test=".='result'">HRESULT</xsl:when>
        <xsl:when test=".='boolean'">BOOL</xsl:when>
        <xsl:when test=".='octet'">BYTE</xsl:when>
        <xsl:when test=".='short'">SHORT</xsl:when>
        <xsl:when test=".='unsigned short'">USHORT</xsl:when>
        <xsl:when test=".='long'">LONG</xsl:when>
        <xsl:when test=".='long long'">LONG64</xsl:when>
        <xsl:when test=".='unsigned long'">ULONG</xsl:when>
        <xsl:when test=".='char'">CHAR</xsl:when>
        <xsl:when test=".='string'">CHAR *</xsl:when>
        <xsl:when test=".='wchar'">OLECHAR</xsl:when>
        <xsl:when test=".='wstring'">BSTR</xsl:when>
        <!-- UUID type -->
        <xsl:when test=".='uuid'">GUID</xsl:when>
        <!-- system interface types -->
        <xsl:when test=".='$unknown'">IUnknown *</xsl:when>
        <xsl:when test=".='unsigned long long'">
          <xsl:message terminate="yes">
            <xsl:value-of select="'&quot;unsigned long long&quot; no longer supported'" />
          </xsl:message>
        </xsl:when>
        <xsl:otherwise>
          <xsl:choose>
            <!-- enum types -->
            <xsl:when test="
              (ancestor::library/enum[@name=current()]) or
              (ancestor::library/if[@target=$self_target]/enum[@name=current()])
            ">
              <xsl:value-of select="."/>
            </xsl:when>
            <!-- custom interface types -->
            <xsl:when test="
              ((ancestor::library/interface[@name=current()]) or
               (ancestor::library/if[@target=$self_target]/interface[@name=current()])
              )
            ">
              <xsl:value-of select="."/><xsl:text> *</xsl:text>
            </xsl:when>
            <!-- other types -->
            <xsl:otherwise>
              <xsl:message terminate="yes">
                <xsl:text>Unknown parameter type: </xsl:text>
                <xsl:value-of select="."/>
              </xsl:message>
            </xsl:otherwise>
          </xsl:choose>
        </xsl:otherwise>
      </xsl:choose>
    </xsl:otherwise>
  </xsl:choose>
</xsl:template>

</xsl:stylesheet>
