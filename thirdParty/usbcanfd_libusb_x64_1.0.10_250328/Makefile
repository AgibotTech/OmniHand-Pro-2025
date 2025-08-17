all: clean test upgrade

test:
	${CC} -o test ${CFLAGS} test.c ${LDFLAGS} -L. -L.. -lpthread -lusb-1.0 -lusbcanfd
	${CC} -o testLin ${CFLAGS} testLin.c ${LDFLAGS} -L. -L.. -lpthread -lusb-1.0 -lusbcanfd
	${CC} -o test_uds ${CFLAGS} test_uds.c ${LDFLAGS} -L. -L.. -lpthread -lusb-1.0 -lusbcanfd

upgrade:
	${CC} -o upgrade ${CFLAGS} upgrade.c ${LDFLAGS} -L. -L.. -lpthread -lusb-1.0 -lusbcanfd

clean:
	rm -vf test
	rm -vf testLin
	rm -vf test_uds
	rm -vf upgrade 

