<?xml version="1.0"?>

<!--
 *  A template to generate a WiX include file that contains
 *  type library definitions for VirtualBox COM components
 *  from the generic interface definition expressed in XML.

    Copyright (C) 2007-2012 Oracle Corporation

    This file is part of VirtualBox Open Source Edition (OSE), as
    available from http://www.virtualbox.org. This file is free software;
    you can redistribute it and/or modify it under the terms of the GNU
    General Public License (GPL) as published by the Free Software
    Foundation, in version 2 as it comes in the "COPYING" file of the
    VirtualBox OSE distribution. VirtualBox OSE is distributed in the
    hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
-->

<xsl:stylesheet version="1.0"
                xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

<xsl:output method="xml"
            version="1.0"
            encoding="utf-8"
            indent="yes"/>

<xsl:strip-space elements="*"/>


<!--
//  templates
/////////////////////////////////////////////////////////////////////////////
-->

<!--
 *  header
-->
<xsl:template match="/idl">
  <xsl:comment>
/*
 *  DO NOT EDIT! This is a generated file.
 *
 *  WiX include script for the VirtualBox Type Library
 *  generated from XIDL (XML interface definition).
 *
 *  Source    : src/VBox/Main/idl/VirtualBox.xidl
 *  Generator : src/VBox/Installer/VirtualBox_TypeLib.xsl
 */
  </xsl:comment>
  <xsl:apply-templates/>
</xsl:template>


<!--
 *  libraries
-->
<xsl:template match="idl/library">
  <Include>
    <TypeLib>
      <xsl:attribute name="Id"><xsl:value-of select="@uuid"/></xsl:attribute>
      <xsl:attribute name="Advertise">yes</xsl:attribute>
      <xsl:attribute name="MajorVersion">1</xsl:attribute>
      <xsl:attribute name="MinorVersion">0</xsl:attribute>
      <xsl:attribute name="Language">0</xsl:attribute>
      <xsl:attribute name="Description"><xsl:value-of select="@desc"/></xsl:attribute>
      <AppId>
        <xsl:attribute name="Id"><xsl:value-of select="@appUuid"/></xsl:attribute>
        <xsl:attribute name="Description"><xsl:value-of select="@name"/> Application</xsl:attribute>
        <xsl:apply-templates select="module/class"/>
      </AppId>
    </TypeLib>
  </Include>
</xsl:template>


<!--
 *  classes
-->
<xsl:template match="library//module/class">
  <Class>
    <xsl:attribute name="Id"><xsl:value-of select="@uuid"/></xsl:attribute>
    <xsl:attribute name="Description"><xsl:value-of select="@name"/> Class</xsl:attribute>
    <xsl:attribute name="Server"><xsl:value-of select="../@name"/></xsl:attribute>
    <xsl:attribute name="Context">
      <xsl:choose>
        <xsl:when test="../@context='InprocServer'">InprocServer32</xsl:when>
        <xsl:when test="../@context='LocalServer'">LocalServer32</xsl:when>
        <xsl:otherwise>
          <xsl:message terminate="yes">
            <xsl:value-of select="concat(../../@name,'::',../@name,': ')"/>
            <xsl:text>module context </xsl:text>
            <xsl:value-of select="concat('&quot;',../@context,'&quot;')"/>
            <xsl:text> is invalid!</xsl:text>
          </xsl:message>
        </xsl:otherwise>
      </xsl:choose>
    </xsl:attribute>
    <xsl:if test="../@context='InprocServer'">
      <xsl:variable name="tmodel" select="(./@threadingModel | ../@threadingModel)[last()]"/>
      <xsl:attribute name="ThreadingModel">
        <xsl:choose>
          <xsl:when test="$tmodel='Apartment'">apartment</xsl:when>
          <xsl:when test="$tmodel='Free'">free</xsl:when>
          <xsl:when test="$tmodel='Both'">both</xsl:when>
          <xsl:when test="$tmodel='Neutral'">neutral</xsl:when>
          <xsl:when test="$tmodel='Single'">single</xsl:when>
          <xsl:when test="$tmodel='Rental'">rental</xsl:when>
          <xsl:otherwise>
            <xsl:message terminate="yes">
              <xsl:value-of select="concat(../../@name,'::',@name,': ')"/>
              <xsl:text>class (or module) threading model </xsl:text>
              <xsl:value-of select="concat('&quot;',$tmodel,'&quot;')"/>
              <xsl:text> is invalid!</xsl:text>
            </xsl:message>
          </xsl:otherwise>
        </xsl:choose>
      </xsl:attribute>
    </xsl:if>
    <ProgId>
      <xsl:attribute name="Id">
        <xsl:value-of select="concat(//library/@name,'.',@name,'.1')"/>
      </xsl:attribute>
      <xsl:attribute name="Description"><xsl:value-of select="@name"/> Class</xsl:attribute>
      <ProgId>
        <xsl:attribute name="Id">
          <xsl:value-of select="concat(//library/@name,'.',@name)"/>
        </xsl:attribute>
        <xsl:attribute name="Description"><xsl:value-of select="@name"/> Class</xsl:attribute>
      </ProgId>
    </ProgId>
  </Class>
</xsl:template>


<!--
 *  eat everything else not explicitly matched
-->
<xsl:template match="*">
</xsl:template>


</xsl:stylesheet>
