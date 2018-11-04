IDIR =/lib/boost/boost_1_68_0/boost
CC=gcc
CFLAGS=-I$(IDIR)
DEPS = SerialPort.h
OBJ = Serial_DMR_5200.o SerialPort.o
ODIR = obj


$(ODIR)/%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

Serial_DMR_5200: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~
