#!/usr/bin/env bash
# Lance les tests du depot avec mesure de couverture Python, comme la CI.
#
#   ./tests/run_tests.sh            tests unitaires + integration + rapport
#   ./tests/run_tests.sh unit       tests unitaires seuls
#   ./tests/run_tests.sh integration  tests d'integration seuls (vrais noeuds ROS)
#   ./tests/run_tests.sh report     fusion des donnees et rapport (HTML, XML, texte)
#
# Prerequis : ROS 2 Jazzy et le workspace sources (source install/setup.bash).
# Les tests d'integration utilisent un domaine ROS dedie et une decouverte limitee
# a la machine : ils n'interferent pas avec une simulation qui tournerait a cote.
#
# Couverture : les tests unitaires tournent sous coverage.py ; pour les tests
# d'integration, seuls les noeuds (sous-processus) sont instrumentes
# (FAUCON_COVERAGE=1) : instrumenter aussi pytest y ajoutait des latences qui
# faussaient les mesures de temps des tests (arret d'urgence, delai de securite).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"
export PYTHONDONTWRITEBYTECODE=1
export COVERAGE_FILE="${COVERAGE_FILE:-$REPO/.coverage}"
export FAUCON_TEST_LOGS="${FAUCON_TEST_LOGS:-$REPO/test-results/logs}"
RESULTS="$REPO/test-results"
mkdir -p "$RESULTS" "$FAUCON_TEST_LOGS"

UNIT_DIRS="faucon_common/test faucon_navigation/test faucon_control/test faucon_base_desc/test"

run_unit() {
    python3 -m coverage run --rcfile=.coveragerc -m pytest -q -p no:cacheprovider \
        --junitxml="$RESULTS/unit.xml" $UNIT_DIRS
}

run_integration() {
    FAUCON_COVERAGE=1 python3 -m pytest -q -p no:cacheprovider \
        --junitxml="$RESULTS/integration.xml" tests/integration
}

report() {
    python3 -m coverage combine --rcfile=.coveragerc -q || true
    python3 -m coverage report --rcfile=.coveragerc
    python3 -m coverage html --rcfile=.coveragerc -q
    python3 -m coverage xml --rcfile=.coveragerc -q
    echo "Rapport HTML : $REPO/coverage_html/index.html"
}

case "${1:-all}" in
    unit) run_unit ;;
    integration) run_integration ;;
    report) report ;;
    all)
        python3 -m coverage erase --rcfile=.coveragerc
        status=0
        run_unit || status=$?
        run_integration || status=$?
        report
        exit $status
        ;;
    *) echo "usage : $0 [all|unit|integration|report]" >&2; exit 2 ;;
esac
