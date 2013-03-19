all:
	g++ -o remote_tester main.cpp -g -Wall

clean:
	rm remote_tester
	rm remote_data.log

run:
	./remote_tester 10.162.242.114 56000 >> remote_data.log
