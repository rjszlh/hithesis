# Makefile for hithesis dissertation

# Compiling method: latexmk/xelatex
METHOD = xelatex
# Set opts for latexmk if you use it
LATEXMKOPTS = -xelatex
# Basename of thesis
THESISMAIN = thesis
PACKAGE = hithesis
XELATEX = xelatex
BIBTEX = bibtex
SPLITINDEX = splitindex
LATEXMK = latexmk

# Optional override for environments that need explicit TeX Live bin path.
# Example:
#   make TEXLIVE_WIN_BIN="E:/texlive/2026/bin/windows"
ifneq ($(strip $(TEXLIVE_WIN_BIN)),)
	XELATEX = "$(TEXLIVE_WIN_BIN)/xelatex.exe"
	BIBTEX = "$(TEXLIVE_WIN_BIN)/bibtex.exe"
	SPLITINDEX = "$(TEXLIVE_WIN_BIN)/splitindex.exe"
	LATEXMK = "$(TEXLIVE_WIN_BIN)/latexmk.exe"
endif

THESISCONTENTS=$(THESISMAIN).tex front/*.tex body/*.tex back/*.tex $(FIGURES)
# NOTE: update this to reflect your local file types.
FIGURES=$(wildcard figures/*.eps figures/*.pdf)
BIBFILE=*.bib
CLSFILES=$(PACKAGE)book.cls $(PACKAGE)book.cfg $(PACKAGE).ist $(PACKAGE).sty *.bst

.DEFAULT_GOAL := thesis

# make deletion work on Windows
ifdef SystemRoot
	RM = del /Q
	OPEN = start
else
	RM = rm -f
	OPEN = xdg-open
endif

.PHONY: clean cleanall thesis viewthesis

viewthesis: thesis
	$(OPEN) $(THESISMAIN).pdf

thesis: $(THESISMAIN).pdf

ifeq ($(METHOD),latexmk)

$(THESISMAIN).pdf: $(CLSFILES)
	$(LATEXMK) $(LATEXMKOPTS) $(THESISMAIN)

else ifeq ($(METHOD),xelatex)

$(THESISMAIN).idx: $(THESISMAIN).bbl
	$(XELATEX) $(THESISMAIN)
	$(XELATEX) $(THESISMAIN)

$(THESISMAIN)_china.idx : $(CLSFILES) $(THESISMAIN).bbl $(THESISMAIN).idx
	$(SPLITINDEX) $(THESISMAIN) -- -s $(PACKAGE).ist  # 自动生成索引

$(THESISMAIN)_english.ind $(THESISMAIN)_china.ind $(THESISMAIN)_english.idx : $(THESISMAIN)_china.idx

$(THESISMAIN).pdf: $(CLSFILES) $(THESISCONTENTS) $(THESISMAIN)_china.ind $(THESISMAIN)_china.idx $(THESISMAIN)_english.ind $(THESISMAIN)_english.idx $(THESISMAIN).bbl
	$(XELATEX) $(THESISMAIN)
	$(SPLITINDEX) $(THESISMAIN) -- -s $(PACKAGE).ist  # 自动生成索引
	$(XELATEX) $(THESISMAIN)

$(THESISMAIN).bbl: $(BIBFILE)
	$(XELATEX) $(THESISMAIN)
	-$(BIBTEX) $(THESISMAIN)
	$(RM) $(THESISMAIN).pdf

else
$(error Unknown METHOD: $(METHOD))

endif

clean:
	$(LATEXMK) -c $(THESISMAIN)
	-@$(RM) *~ *.idx *.ind *.ilg *.thm *.toe *.bbl

cleanall: clean
	-@$(RM) $(THESISMAIN).pdf
