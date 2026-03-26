CFLAGS = -O3
LIBS   = -lpthread
CPP    = g++ $(CFLAGS) -c
SRCDIR = src

# Evolution executable
Comm: $(SRCDIR)/main.o $(SRCDIR)/CommAgent1D.o $(SRCDIR)/SensorCTRNN.o $(SRCDIR)/Search.o $(SRCDIR)/random.o
	g++ $(LIBS) $(CFLAGS) $^ -o Comm

# Analysis executable
Analysis: $(SRCDIR)/AnalysisMain.o $(SRCDIR)/CommAgent1D.o $(SRCDIR)/SensorCTRNN.o $(SRCDIR)/random.o
	g++ $(CFLAGS) $^ -o Analysis

$(SRCDIR)/main.o: $(SRCDIR)/main.cpp $(SRCDIR)/CommAgent1D.h $(SRCDIR)/SensorLoadCircuit.h $(SRCDIR)/Search.h $(SRCDIR)/globals.h
	$(CPP) $(SRCDIR)/main.cpp -o $@

$(SRCDIR)/AnalysisMain.o: $(SRCDIR)/AnalysisMain.cpp $(SRCDIR)/CommAgent1D.h $(SRCDIR)/globals.h
	$(CPP) $(SRCDIR)/AnalysisMain.cpp -o $@

$(SRCDIR)/CommAgent1D.o: $(SRCDIR)/CommAgent1D.cpp $(SRCDIR)/CommAgent1D.h
	$(CPP) $(SRCDIR)/CommAgent1D.cpp -o $@

$(SRCDIR)/SensorCTRNN.o: $(SRCDIR)/SensorCTRNN.cpp $(SRCDIR)/SensorCTRNN.h $(SRCDIR)/random.h
	$(CPP) $(SRCDIR)/SensorCTRNN.cpp -o $@

$(SRCDIR)/Search.o: $(SRCDIR)/Search.cpp $(SRCDIR)/Search.h
	$(CPP) $(SRCDIR)/Search.cpp -o $@

$(SRCDIR)/random.o: $(SRCDIR)/random.cpp $(SRCDIR)/random.h
	$(CPP) $(SRCDIR)/random.cpp -o $@

clean:
	rm -f Comm Analysis $(SRCDIR)/*.o

.PHONY: clean
