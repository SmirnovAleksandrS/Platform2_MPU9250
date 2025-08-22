# =========================
# export.mk (simple, non-recursive)
# =========================
LIBNAME := MPU9250_LIB      # <-- ПОДМЕНИ: INNER_PROTO / MPU9250_LIB / QMC5883_LIB

LIB := $(strip $(LIBNAME))
LIB_DIR := $($(LIB)_DIR)

# Диагностика — оставить на время
$(info [$(LIB)] DIR=$(LIB_DIR))

# Папки
INC_ROOT := $(LIB_DIR)/Inc
SRC_ROOT := $(LIB_DIR)/Src

# Includes: Inc/ + её подпапки первого уровня (без дублей)
$(LIB)_INC_DIRS := $(wildcard $(INC_ROOT)) $(wildcard $(INC_ROOT)/*)
$(LIB)_INCLUDES := $(addprefix -I,$(sort $($(LIB)_INC_DIRS)))

# Sources: только файлы в Src/ (без подпапок)
$(LIB)_SRCS := $(wildcard $(SRC_ROOT)/*.c) $(wildcard $(SRC_ROOT)/*.s) $(wildcard $(SRC_ROOT)/*.S)

# Экспорт
ifneq ($(wildcard $(INC_ROOT)),)
  C_INCLUDES += $($(LIB)_INCLUDES)
endif
ifneq ($(wildcard $(SRC_ROOT)),)
  C_SOURCES  += $($(LIB)_SRCS)
endif
