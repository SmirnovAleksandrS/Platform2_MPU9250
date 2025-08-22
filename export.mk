# ---- export.mk (strict, non-recursive, guarded) ----
# УСТАНОВИ ПРАВИЛЬНО LIBNAME для КАЖДОЙ ЛИБЫ!
LIBNAME := MPU9250_LIB    # <- в mpu9250-lib поставить MPU9250_LIB, в QMC поставить QMC5883_LIB

# -------- include-guard --------
LIB := $(strip $(LIBNAME))
ifndef $(LIB)_EXPORT_INCLUDED
$(LIB)_EXPORT_INCLUDED := 1

# -------- базовые пути от generated/third_party.mk --------
LIB_DIR := $($(LIB)_DIR)

# Проверки (дадут понятную ошибку, если что-то не так)
ifeq ($(strip $(LIB_DIR)),)
  $(error [$(LIB)] $($(LIB)_DIR) is empty. Check LIBNAME and generated/third_party.mk)
endif

INC_ROOT := $(LIB_DIR)/Inc
SRC_ROOT := $(LIB_DIR)/Src

ifeq ($(wildcard $(INC_ROOT))$(wildcard $(SRC_ROOT)),)
  $(error [$(LIB)] Expected $(INC_ROOT) and $(SRC_ROOT) to exist. Found: INC=$(wildcard $(INC_ROOT)) SRC=$(wildcard $(SRC_ROOT)))
endif

# -------- добавляем пути и исходники (БЕЗ рекурсии) --------
C_INCLUDES += -I$(INC_ROOT)
C_SOURCES  += $(wildcard $(SRC_ROOT)/*.c) $(wildcard $(SRC_ROOT)/*.s) $(wildcard $(SRC_ROOT)/*.S)

# Отладочный вывод (оставь пока)
$(info [$(LIB)] DIR=$(LIB_DIR))
$(info [$(LIB)] +I=$(INC_ROOT))
$(info [$(LIB)] SRCS=$(wildcard $(SRC_ROOT)/*.c) $(wildcard $(SRC_ROOT)/*.s) $(wildcard $(SRC_ROOT)/*.S))

endif  # include-guard
