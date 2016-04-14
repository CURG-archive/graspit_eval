
#to build as a make file in windows, change vclib to lib
win32{
TEMPLATE	= vclib
}else{
TEMPLATE = lib
}

LANGUAGE	= C++
CONFIG += qt dylib
QMAKE_CXXFLAGS += -std=c++11
QT += qt3support
DESTDIR = lib
MOC_DIR = build
OBJECTS_DIR = build

!exists($(GRASPIT)) {
   error("GRASPIT environment variable not set")
}

INCLUDEPATH += $(GRASPIT) $(GRASPIT)/qjson4 $(GRASPIT)/cmdline

# Mongo Driver headers
INCLUDEPATH += /home/timchunght/graspit_data_gen/mongo-cxx-driver/build/install/include/mongocxx/v_noabi
INCLUDEPATH += /home/timchunght/graspit_data_gen/mongo-cxx-driver/build/install/include/bsoncxx/v_noabi

DEPENDPATH += $(GRASPIT)/src 

HEADERS += $(GRASPIT)/include/plugin.h \
    graspGenerationPlugin.h

SOURCES += \
    main.cpp \
    graspGenerationPlugin.cpp

# Mongo Driver shared lib
LIBS += -L/home/timchunght/graspit_data_gen/mongo-cxx-driver/build/install/lib -lmongocxx
