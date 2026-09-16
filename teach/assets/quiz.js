/* Reusable quiz widget — retrieval practice with immediate, automatic feedback.
   Usage in a lesson:

   <div class="quiz" data-answer="B">
     <p class="q">Question text?</p>
     <button data-opt="A">First option</button>
     <button data-opt="B">Second option</button>
     <button data-opt="C">Third option</button>
     <p class="explain" hidden>Why B is right…</p>
   </div>

   Load once per page: <script src="../assets/quiz.js" defer></script>
   Answers should be equal-length options so formatting gives nothing away. */

(function () {
  function styleInject() {
    if (document.getElementById('quiz-style')) return;
    var css = `
    .quiz{font-family:var(--font-sans);background:var(--card);border:1px solid var(--rule);
      border-radius:8px;padding:1.1rem 1.2rem;margin:1.6rem 0;box-shadow:var(--shadow)}
    .quiz .q{font-weight:650;margin:.1rem 0 .9rem;font-size:1rem}
    .quiz button{display:block;width:100%;text-align:left;font:inherit;font-size:.92rem;
      background:var(--bg);color:var(--ink);border:1px solid var(--rule);border-radius:6px;
      padding:.6rem .8rem;margin:.4rem 0;cursor:pointer;transition:border-color .15s,background .15s}
    .quiz button:hover{border-color:var(--accent)}
    .quiz button.correct{border-color:var(--ok);background:color-mix(in srgb,var(--ok) 12%,transparent);font-weight:650}
    .quiz button.wrong{border-color:var(--bad);background:color-mix(in srgb,var(--bad) 12%,transparent)}
    .quiz button:disabled{cursor:default;opacity:.85}
    .quiz .explain{margin:.8rem 0 .2rem;font-size:.9rem;line-height:1.55;
      background:var(--code-bg);border-radius:6px;padding:.7rem .9rem}
    .quiz .verdict{font-weight:700;font-size:.85rem;margin:.6rem 0 0}
    .quiz .verdict.ok{color:var(--ok)} .quiz .verdict.no{color:var(--bad)}`;
    var s = document.createElement('style');
    s.id = 'quiz-style'; s.textContent = css;
    document.head.appendChild(s);
  }

  function wire(quiz) {
    var answer = quiz.getAttribute('data-answer');
    var explain = quiz.querySelector('.explain');
    var buttons = quiz.querySelectorAll('button[data-opt]');
    var verdict = document.createElement('p');
    verdict.className = 'verdict';
    buttons.forEach(function (btn) {
      btn.addEventListener('click', function () {
        var chosen = btn.getAttribute('data-opt');
        buttons.forEach(function (b) {
          b.disabled = true;
          var opt = b.getAttribute('data-opt');
          if (opt === answer) b.classList.add('correct');
          else if (opt === chosen) b.classList.add('wrong');
        });
        var right = chosen === answer;
        verdict.textContent = right ? '✓ 正确' : '✗ 再想想 —— 正确答案已高亮';
        verdict.classList.add(right ? 'ok' : 'no');
        if (explain) { explain.hidden = false; quiz.insertBefore(verdict, explain); }
        else quiz.appendChild(verdict);
      });
    });
  }

  document.addEventListener('DOMContentLoaded', function () {
    styleInject();
    document.querySelectorAll('.quiz').forEach(wire);
  });
})();
