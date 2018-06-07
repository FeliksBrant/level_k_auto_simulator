##############################################################################
#
#    file                 : Makefile
#    created              : Tue Jan 9 02:50:43 EST 2018
#    copyright            : (C) 2002 Guankun Su
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = level1_1
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}

lkMODULE	= LevelKDriver policy parameters record cardata opponent gui guifont
lkSRC		= $(addsuffix .cpp,$(lkMODULE))
lkLIB		= $(addsuffix .h,$(lkMODULE))
lkPATH 		= ../levelK/

SOURCES     = ${ROBOT}.cpp driver.cpp $(lkSRC)

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml car1-trb1.rgb logo.rgb
SHIPSUBDIRS = 0 1 2 3 4 5 6 7 8 9

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-level1_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-level1_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}

build: copy default install remove
	
copy:
	cp $(addprefix $(lkPATH),$(lkSRC)) .
	cp $(addprefix $(lkPATH),$(lkLIB)) .	

remove:
	rm $(lkSRC) $(lkLIB)


