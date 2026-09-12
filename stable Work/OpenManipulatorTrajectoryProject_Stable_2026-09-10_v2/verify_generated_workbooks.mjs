import fs from "node:fs/promises";
import path from "node:path";
import { FileBlob, SpreadsheetFile } from "file:///C:/Users/slima/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/@oai/artifact-tool/dist/artifact_tool.mjs";

const inputDir = process.argv[2] ?? "calibration_rebuild/outputs/controller_workbook_smoke";
const previewDir = path.join(inputDir, "verification");
const files = ["manual_poses.xlsx", "trajectory_full.xlsx", "trajectory_xyz.xlsx"];

await fs.mkdir(previewDir, { recursive: true });
for (const fileName of files) {
  const workbookPath = path.resolve(inputDir, fileName);
  const blob = await FileBlob.load(workbookPath);
  const workbook = await SpreadsheetFile.importXlsx(blob);
  const summary = await workbook.inspect({
    kind: "sheet,table",
    maxChars: 4000,
    tableMaxRows: 6,
    tableMaxCols: 14,
  });
  console.log(`${fileName}\n${summary.ndjson}`);
  const errors = await workbook.inspect({
    kind: "match",
    searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A|#NUM!|#NULL!|#SPILL!|#CALC!",
    options: { useRegex: true, maxResults: 100 },
    summary: `${fileName} formula error scan`,
  });
  console.log(errors.ndjson);
  const sheetInfo = await workbook.inspect({ kind: "sheet", include: "id,name", maxChars: 1000 });
  const sheetRecord = JSON.parse(sheetInfo.ndjson.trim().split("\n")[0]);
  const sheetName = sheetRecord.name ?? sheetRecord.sheetName;
  const preview = await workbook.render({ sheetName, autoCrop: "all", scale: 1, format: "png" });
  await fs.writeFile(
    path.join(previewDir, fileName.replace(".xlsx", ".png")),
    new Uint8Array(await preview.arrayBuffer()),
  );
}
