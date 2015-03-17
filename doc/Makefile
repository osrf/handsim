#LATEX = latex
LATEX = pdflatex
BIBTEX = bibtex
DVIPS = dvips
PS2PDF = ps2pdf
VIEWPDF = okular

BASE_NAME = referenceguide
OPTITRACK_NAME = optitrack

OTHER_FILES = 

all: $(BASE_NAME).pdf 
transforms: $(OPTITRACK_NAME).pdf 

%.pdf: %.tex $(OTHER_FILES) $(PNG_FILES) 
	$(LATEX) $< && $(LATEX) $< && $(LATEX) $<

%.dvi: %.tex
	$(LATEX) $< && $(LATEX) $< && $(LATEX) $<

%.ps: %.dvi
	$(DVIPS) -P pdf -f -t a4 $<.dvi > $@

display: $(BASE_NAME).pdf
	$(VIEWPDF) $(BASE_NAME).pdf &

%.eps: %.dia
	cd `dirname $<` && dia `basename $<` -t eps

%.pdf: %.eps
	epstopdf $<

clean:
	rm -rf $(BASE_NAME).ps $(BASE_NAME).dvi $(BASE_NAME).log \
	      $(BASE_NAME).aux $(BASE_NAME).blg $(BASE_NAME).toc \
	      missfont.log $(BASE_NAME).bbl $(BASE_NAME).pdf \
				doxygen html latex $(BASE_NAME).out
