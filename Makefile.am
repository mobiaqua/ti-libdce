## Process this file with automake to produce Makefile.in

SUBDIRS                      = \
                               packages/codec_engine \
                               packages/ivahd_codecs \
                               packages/xdais \
                               packages/xdctools \
                               packages/framework_components


lib_LTLIBRARIES              = libdce.la


CE_CFLAGS                    = \
                               -I$(top_srcdir)/packages/codec_engine \
                               -I$(top_srcdir)/packages/ivahd_codecs \
                               -I$(top_srcdir)/packages/xdais \
                               -I$(top_srcdir)/packages/xdctools \
                               -I$(top_srcdir)/packages/framework_components \
                               `pkg-config --cflags libmmrpc` \
			       -DBUILDOS_LINUX=1 -DDCE_DEBUG_LEVEL=1 \
			       -DDCE_DEBUG_ENABLE=1 \
                               -Wno-pointer-to-int-cast


libdce_la_SOURCES            = libdce.c memplugin_linux.c libdce_linux.c
libdce_la_CFLAGS             = $(WARN_CFLAGS) $(CE_CFLAGS) $(DRM_CFLAGS)
libdce_la_LDFLAGS            = -no-undefined -version-info 1:0:0 `pkg-config --libs libmmrpc`
libdce_la_LIBADD             = $(DRM_LIBS)

libdce_la_includedir         = $(includedir)/dce
libdce_la_include_HEADERS    = libdce.h

pkgconfig_DATA               = libdce.pc
pkgconfigdir                 = $(libdir)/pkgconfig
