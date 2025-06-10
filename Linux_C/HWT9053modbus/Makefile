# Compiler to use
CXX = g++

# Compiler flags. -Wall and -Wextra are highly recommended for catching errors.
# We'll handle -fpermissive on a per-file basis.
CXXFLAGS = -Wall -Wextra

# The final executable name
TARGET = hwt9053_node

# List of all object files needed to build the target
# Make will automatically deduce that .o files come from .c files
OBJS = main_pc.o serial.o wit_c_sdk.o

# The default goal of this Makefile is to build the 'all' target.
# It's a good practice to have this.
all: $(TARGET)

# --- Rules ---

# Rule to link all the object files into the final executable
$(TARGET): $(OBJS)
	@echo "Linking..."
	$(CXX) $(OBJS) -o $(TARGET)
	@echo "Build finished: $(TARGET)"

# Specific rule for serial.o because it needs a special flag
serial.o: serial.c
	@echo "Compiling $< with -fpermissive..."
	$(CXX) $(CXXFLAGS) -fpermissive -c $< -o $@

# Generic rule to compile .c source files into .o object files.
# Make will use this for newmain.c and wit_c_sdk.c.
# '$<' is the prerequisite (the .c file)
# '$@' is the target (the .o file)
%.o: %.c
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 'Phony' target to clean up the directory
# It removes the executable and all object files.
.PHONY: clean
clean:
	@echo "Cleaning up..."
	rm -f $(TARGET) $(OBJS)
