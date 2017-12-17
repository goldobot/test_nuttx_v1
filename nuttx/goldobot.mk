.DEFAULT_GOAL = all 
.SILENT:

TARGET := $(BIN).bin

$(TARGET): $(BIN)

flash: $(TARGET)
	$(Q)echo -n "flashing target  "
	$(Q)cp $< /media/zakaria/NODE_F303RE/
	$(Q)echo "..... done"
