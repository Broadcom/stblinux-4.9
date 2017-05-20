INCLUDEDIR		= -I/usr/include/ncurses

OBJS			= input.o menurt.o listbox.o winops.o labels.o \
				msgboxes.o txbox.o

libtextbox.a: $(OBJS)
	rm -rf libtextbox.a
	$(AR) cq libtextbox.a $(OBJS)
	$(RANLIB) libtextbox.a
#	$(CC) -shared -o libtextbox.so $(OBJS)

%.o: %.c *.h
	$(CC) $(CFLAGS) -Wall -fPIC $(INCLUDEDIR) -c -o $*.o $<

clean:
	rm -rf *.o *~ libtextbox.a libtextbox.so


