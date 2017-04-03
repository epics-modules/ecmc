
# Extra stuff to show which version we run
build: generategitversion

generategitversion:
	tools/gitversion.sh ecmcApp/src/gitversion.c


#Use either the Makefile for EPICS, or the one for
# ESS EPICS ENVIRONMENT

ifdef EPICS_ENV_PATH
ifeq ($(EPICS_MODULES_PATH),/opt/epics/modules)
ifeq ($(EPICS_BASES_PATH),/opt/epics/bases)
include Makefile.EEE
else
include Makefile.epics
endif
else
include Makefile.epics
endif
else
include Makefile.epics
endif
