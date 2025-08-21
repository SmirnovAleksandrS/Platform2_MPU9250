# =========================
# Auto-export for IMU driver
# =========================
# Имя библиотеки КАПСОМ. ДОЛЖНО совпадать с тем, что сгенерит configure.py:
#   name: "imu_driver"  -> var: IMU_DRIVER
LIBNAME := INNER_PROTO

# Путь до либы приходит из основного проекта:
#   IMU_DRIVER_DIR := <...>   (создаётся generated/third_party.mk)
LIB_DIR := $($(LIBNAME)_DIR)

INC_ROOT := $(LIB_DIR)/Inc
SRC_ROOT := $(LIB_DIR)/Src

# --- Файлы (рекурсивно): все .c, .s/.S и директории с заголовками ---
# Требуется 'find'. Если его нет, см. "Вариант без find" ниже.

# Все исходники C/ASM под Src/**
$(LIBNAME)_SRCS := $(shell find "$(SRC_ROOT)" -type f \( -name '*.c' -o -name '*.s' -o -name '*.S' \) 2>/dev/null)

# Все директории, где лежат хедеры, под Inc/**
# 1) найти все *.h/*.hpp, 2) взять их директории, 3) уникализировать, 4) превратить в -I...
$(LIBNAME)_HDRS := $(shell find "$(INC_ROOT)" -type f \( -name '*.h' -o -name '*.hpp' \) 2>/dev/null)
$(LIBNAME)_INC_DIRS := $(sort $(dir $($(LIBNAME)_HDRS)))
$(LIBNAME)_INCLUDES := $(addprefix -I,$($(LIBNAME)_INC_DIRS))

# --- Опциональные дефайны/доп.линковка (правь по мере надобности) ---
$(LIBNAME)_DEFS :=
$(LIBNAME)_LIBS :=

# --- Проверки существования каталогов (чтобы make не ругался на пустые) ---
ifneq ($(wildcard $(INC_ROOT)),)
  C_INCLUDES += $($(LIBNAME)_INCLUDES)
endif
ifneq ($(wildcard $(SRC_ROOT)),)
  C_SOURCES  += $($(LIBNAME)_SRCS)
endif

C_DEFS += $($(LIBNAME)_DEFS)
LIBS   += $($(LIBNAME)_LIBS)

# --- Отладочный вывод (раскомментируй при необходимости) ---
# $(info [$(LIBNAME)] INC: $($(LIBNAME)_INCLUDES))
# $(info [$(LIBNAME)] SRC: $($(LIBNAME)_SRCS))
