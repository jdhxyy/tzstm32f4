TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += ../../ \
    ../../lib/tzmalloc \
    ../../lib/tztype-clang \
    ../../lib/tzcomponent-clang

SOURCES += \
        main.c \
    ../../lib/tzmalloc/bget.c \
    ../../lib/tzmalloc/tzmalloc.c \
    ../../tzstm32f4io.c \
    ../../tzstm32f4iwdg.c \
    ../../tzstm32f4uart1.c \
    ../../tzstm32f4uart3.c \
    ../../tzstm32f4uart4.c \
    ../../tzstm32f4uart6.c \
    ../../tzstm32f4uartcom.c

HEADERS += \
    ../../lib/tzcomponent-clang/tzio.h \
    ../../lib/tzcomponent-clang/tziwdg.h \
    ../../lib/tzmalloc/bget.h \
    ../../lib/tzmalloc/tzmalloc.h \
    ../../lib/tztype-clang/tztype.h \
    ../../tzstm32f4io.h \
    ../../tzstm32f4uart1.h \
    ../../tzstm32f4uart3.h \
    ../../tzstm32f4uart4.h \
    ../../tzstm32f4uart6.h \
    ../../tzstm32f4uartcom.h
