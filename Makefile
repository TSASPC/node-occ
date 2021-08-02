all:
	node-gyp configure
	node-gyp build
	mocha

.PHONY: test
test:
	mocha

clean:
	node-gyp clean

packet:
	npm install mocha
	npm install assert
	npm install should
	npm install node-gyp		
	

copy: 
	COPY D:\projet\oce-build\bin\Debug\*.dll build\Release

format:
	astyle --indent=spaces=4 src/*
